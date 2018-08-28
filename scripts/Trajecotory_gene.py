
from trajectory_generation import Trajectory_generator


if __name__ == "__main__":
    trajectory_generate = Trajectory_generator(node_name = "Traj_gene_node")

    while not rospy.is_shutdown():
        rospy.spin()
