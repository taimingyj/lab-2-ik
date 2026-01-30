import ikpy.chain
import numpy as np
import math
import ikpy.utils.plot as plot_utils

def sphere():
	my_chain = ikpy.chain.Chain.from_urdf_file("./ur5/ur5_gripper.urdf")
	points = []
	for i in range(5000):
		gauss = np.random.normal(0, 1, 3)
		target_position = gauss / math.sqrt(gauss[0] ** 2 + gauss[1] ** 2 + gauss[2] ** 2)
		real_frame = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_position))
		points.append(real_frame[:3, 3])
	points = np.array(points)
	fig, ax = plot_utils.init_3d_figure()
	my_chain.plot(my_chain.inverse_kinematics(target_position), ax)
	ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='red', s=10, label='End-effector positions')
	ax.legend()
	ax.set_box_aspect([1, 1, 1])
	plot_utils.show_figure()

if __name__ == "__main__":
    sphere()