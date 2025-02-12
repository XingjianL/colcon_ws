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
        #builder.load_model(self.base_model, "/home/lxianglabxing/githubrepo/PoinTr/experiments/AdaPoinTr_Plant_Large/PCN_models/shifted_plants/ckpt-best.pth")
        builder.load_model(self.base_model, "/home/lxianglabxing/githubrepo/PoinTr/experiments/AdaPoinTr_Combined/PCN_models/farthest_iros_combined_seg_plant/ckpt-best.pth")
        self.base_model.cuda()
        self.base_model.eval()
        print(sum(p.numel() for p in self.base_model.parameters()))
        print(torch.cuda.mem_get_info())
        self.point_cloud_path = ""
        self.infer_count = 0
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
        self.infer_count += 1
        if save_path != None:
            input_pcd = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(pcd))
            dense_pcd = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(dense_points))
            #pred_path = os.path.join(*[save_path,"pred.pcd"])
            os.makedirs('/'.join(save_path.split('/')[:-1]), exist_ok=True)
            print(f"{save_path}_ModelInput.pcd")
            open3d.io.write_point_cloud(f"{save_path}_{self.infer_count}_ModelInput.pcd", input_pcd)
            open3d.io.write_point_cloud(f"{save_path}_{self.infer_count}_ModelPred.pcd", dense_pcd)

        return dense_points
    
class PartialPCDPredictorNode(Node):
    def __init__(self, predictor_model : AdaPoinTrModel):
        super().__init__("benchbot_nbv")
        self._action_server = ActionServer(
            self,
            PcdNBV,
            'nbv_server',
            self.action_callback
        )
        self.predictor_model = predictor_model
        # self.subscription
        self.received_pcd = 0
        self.NBV = NBV()
    def action_callback(self, goal_handle):
        self.get_logger().info("Action server received partial point cloud")
        feedback_msg = PcdNBV.Feedback()
        feedback_msg.progress = "received"
        goal_handle.publish_feedback(feedback_msg)
        print("received")
        # received and parsing pcd
        pred_input_pcd_ros2 : PointCloud2 = goal_handle.request.pred_input_pcd
        save_prefix = goal_handle.request.save_prefix
        pred_input_pcd = np.frombuffer(pred_input_pcd_ros2.data, dtype=np.uint8).reshape(-1,pred_input_pcd_ros2.point_step)
        pred_input_pcd = pred_input_pcd[:,0:12].view(dtype=np.float32).reshape(-1,3)
        feedback_msg.progress = "parsed input"
        goal_handle.publish_feedback(feedback_msg)
        self.received_pcd += 1
        print(f"input_pcd: {pred_input_pcd.shape}")
        # prediction
        pred = self.predictor_model.inference_single_pcd(pred_input_pcd,f"/home/lxianglabxing/colcon_ws/{save_prefix}")
        feedback_msg.progress = "prediction generated"
        goal_handle.publish_feedback(feedback_msg)
        print(f"pred_pcd: {pred.shape}")
        # next-best-view selection
        nbv_input_pcd_ros2 : PointCloud2 = goal_handle.request.nbv_input_pcd 
        nbv_input_pcd = np.frombuffer(nbv_input_pcd_ros2.data, dtype=np.uint8).reshape(-1,nbv_input_pcd_ros2.point_step)
        nbv_input_pcd = pred_input_pcd[:,0:12].view(dtype=np.float32).reshape(-1,3)
        o3d_input = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(nbv_input_pcd))
        o3d_pred = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(pred))
        self.NBV.set_pcds(o3d_input,o3d_pred)
        feedback_msg.progress = "set input to NBV"
        goal_handle.publish_feedback(feedback_msg)
        view_points, optimal_order, view_center, pred_new_points = self.NBV.generate(const_r=False)
        feedback_msg.progress = "generated NBV"
        goal_handle.publish_feedback(feedback_msg)
        print(f"generated_NBV: {optimal_order}")
        # return result
        result = PcdNBV.Result()
        result.view_points = view_points.flatten().tolist()
        result.optimal_order = optimal_order.tolist()
        result.view_center = view_center.tolist()
        result.num_new_points = pred_new_points
        goal_handle.succeed()
        feedback_msg.progress = "completed"
        goal_handle.publish_feedback(feedback_msg)
        print(f"finished")
        return result


    # def callback(self, msg: PointCloud2):
    #     self.get_logger().info("Received partial point cloud")
    #     pc_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1,msg.point_step)
    #     xyz = pc_data[:,0:12].view(dtype=np.float32).reshape(-1,3)
    #     pred = self.predictor_model.inference_single_pcd(xyz,f"/home/lxianglabxing/colcon_ws/output/pcd/model_io/{self.received_pcd}")
    #     o3d_input = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(xyz))
    #     o3d_pred = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(pred))
    #     self.NBV.set_pcds(o3d_input,o3d_pred)
    #     view_points, optimal_order = self.NBV.generate()
    #     self.received_pcd += 1
        
    #     fields = [
    #         PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    #         PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    #         PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    #     ]
    #     point_list = pred.tolist()        
    #     header = Header()
    #     header.stamp = self.get_clock().now().to_msg()

    #     msg = point_cloud2.create_cloud(header, fields, point_list)
    #     self.publisher.publish(msg)
    #     self.get_logger().info("Sent predicted point cloud")
    #     pass

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
