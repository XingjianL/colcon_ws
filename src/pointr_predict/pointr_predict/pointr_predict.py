import os
import numpy as np
from pointr_minimal.utils.config import cfg_from_yaml_file
from pointr_minimal.tools import builder
from pointr_minimal.datasets.io import IO
from pointr_minimal.datasets.data_transforms import Compose
from next_best_view_gen.nbv_gen import NBV

import torch
import open3d

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from custom_msg.action import PcdNBV
import sys

class AdaPoinTrModel():
    def __init__(self) -> None:
        self.config = cfg_from_yaml_file("/home/lxianglabxing/githubrepo/PoinTr/cfgs/PCN_models/AdaPoinTr_Plant_Large.yaml")
        self.base_model = builder.model_builder(self.config.model)
        builder.load_model(self.base_model, "/home/lxianglabxing/githubrepo/PoinTr/experiments/AdaPoinTr_Plant_Large/PCN_models/shifted_plants/ckpt-best.pth")
        self.base_model.cuda()
        self.base_model.eval()
        print(sum(p.numel() for p in self.base_model.parameters()))
        print(torch.cuda.mem_get_info())
        self.point_cloud_path = ""
        pass
    def inference_single_pcd_file(self, pc_path, save_path = None):
        # read single point cloud
        #pc_ndarray = IO.get(pc_path).astype(np.float32)
        pc = open3d.io.read_point_cloud(pc_path)
        if len(pc.points) > 2000:
            pc = pc.farthest_point_down_sample(2000)
        ptcloud = np.array(pc.points).astype(np.float32)
        print(self.config.dataset.train._base_['NAME'])
        dense_points = self.inference_single_pcd(ptcloud,save_path)
        return dense_points
    def inference_single_pcd(self, pcd, save_path = None):
        transform = Compose([{
            'callback': 'UpSamplePoints',
            'parameters': {
                'n_points': 2048
            },
            'objects': ['input']
        }, {
            'callback': 'ToTensor',
            'objects': ['input']
        }])

        pc_ndarray_normalized = transform({'input': pcd})
        # inference
        ret = self.base_model(pc_ndarray_normalized['input'].unsqueeze(0).to("cuda"))
        dense_points = ret[-1].squeeze(0).detach().cpu().numpy()

        if save_path != None:
            input_pcd = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(pcd))
            dense_pcd = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(dense_points))
            #pred_path = os.path.join(*[save_path,"pred.pcd"])
            open3d.io.write_point_cloud(f"{save_path}input.pcd", input_pcd)
            open3d.io.write_point_cloud(f"{save_path}pred.pcd", dense_pcd)

        return dense_points
    
class PartialPCDPredictorNode(Node):
    def __init__(self, predictor_model : AdaPoinTrModel):
        super().__init__("benchbot_nbv")
        self.subscription = self.create_subscription(
            PointCloud2,
            "/partial_pcd",
            self.callback,
            10
        )
        self.publisher = self.create_publisher(
            PointCloud2,
            "/predicted_pcd",
            10
        )
        self._action_server = ActionServer(
            self,
            PcdNBV,
            'nbv_server',
            self.action_callback
        )
        self.predictor_model = predictor_model
        self.subscription
        self.received_pcd = 0
        self.NBV = NBV()
    def action_callback(self, goal_handle):
        self.get_logger().info("Action server received partial point cloud")
        feedback_msg = PcdNBV.Feedback()
        feedback_msg.progress = "received"
        # received and parsing pcd
        input_pcd_ros2 : PointCloud2 = goal_handle.request.input_pcd 
        input_pcd = np.frombuffer(input_pcd_ros2.data, dtype=np.uint8).reshape(-1,input_pcd_ros2.point_step)
        xyz = input_pcd[:,0:12].view(dtype=np.float32).reshape(-1,3)
        feedback_msg.progress = "parsed input"

        # prediction
        pred = self.predictor_model.inference_single_pcd(xyz,f"/home/lxianglabxing/colcon_ws/output/pcd/model_io/{self.received_pcd}")
        feedback_msg.progress = "prediction generated"

        # next-best-view selection
        o3d_input = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(xyz))
        o3d_pred = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(pred))
        self.NBV.set_pcds(o3d_input,o3d_pred)
        feedback_msg.progress = "set input to NBV"
        view_points, optimal_order = self.NBV.generate()
        feedback_msg.progress = "generated NBV"

        # return result
        result = PcdNBV.Result()
        result.view_points = view_points.flatten().tolist()
        result.optimal_order = optimal_order.tolist()
        goal_handle.succeed()
        feedback_msg.progress = "completed"
        return result


    def callback(self, msg: PointCloud2):
        self.get_logger().info("Received partial point cloud")
        pc_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1,msg.point_step)
        xyz = pc_data[:,0:12].view(dtype=np.float32).reshape(-1,3)
        pred = self.predictor_model.inference_single_pcd(xyz,f"/home/lxianglabxing/colcon_ws/output/pcd/model_io/{self.received_pcd}")
        o3d_input = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(xyz))
        o3d_pred = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(pred))
        self.NBV.set_pcds(o3d_input,o3d_pred)
        view_points, optimal_order = self.NBV.generate()
        self.received_pcd += 1
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        point_list = pred.tolist()        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()

        msg = point_cloud2.create_cloud(header, fields, point_list)
        self.publisher.publish(msg)
        self.get_logger().info("Sent predicted point cloud")
        pass

def main(args = None):
    rclpy.init(args=args)
    model = AdaPoinTrModel()
    pointr_pred_node = PartialPCDPredictorNode(model)
    print('Hi from pointr_predict.')
    rclpy.spin(pointr_pred_node)
    
    rclpy.shutdown()
    #model.inference_single_pcd_file("/home/lxianglabxing/colcon_ws/output/pcd/plant_1726521256.334826/17/S_Bigleaf_Hydrangea_vgztealha_Var10_lod0/1_0_90_0_17.pcd",
    #                            save_path="/home/lxianglabxing/colcon_ws/output/pcd/plant_1726521256.334826/17/S_Bigleaf_Hydrangea_vgztealha_Var10_lod0/")
    import time
    time.sleep(1)
    

if __name__ == '__main__':
    main()
