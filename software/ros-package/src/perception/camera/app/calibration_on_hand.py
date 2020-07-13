import numpy as np
import matplotlib.pyplot as plt
import cv2

def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    
    # centre the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = np.transpose(AA).dot(BB)

    U, S, Vt = np.linalg.svd(H)

    R = Vt.T.dot(U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       print "Reflection detected"
       Vt[2,:] *= -1
       R = Vt.T.dot(U.T)

    t = -R.dot(centroid_A.T) + centroid_B.T

    return R, t

pts_cam = np.array(
[[-0.138869699408 , -0.113655975047,   0.52, 1],
 [-0.114624726136 , -0.0355889437464,  0.52, 1],
 [-0.0368464939632, -0.0599431076998,  0.52, 1],
 [-0.0608742812571, -0.138038452681,   0.52, 1],
 [-0.0911397303553,  0.0428490733517,  0.52, 1],
 [-0.0127405703924,  0.018424685548,   0.52, 1],
 [ 0.0659486553468, -0.00591239562289, 0.52, 1],
 [ 0.0414872480218, -0.0841195499204,  0.52, 1],
 [ 0.0174973642302, -0.162200181166,   0.52, 1]])
pts_cam[:,0] = -pts_cam[:,0]

pts_wld = np.array(
[[-0.08,-0.08, 0.52, 1],
 [-0.08, 0.00, 0.52, 1],
 [ 0.00, 0.00, 0.52, 1],
 [ 0.00,-0.08, 0.52, 1],
 [-0.08, 0.08, 0.52, 1],
 [ 0.00, 0.08, 0.52, 1],
 [ 0.08, 0.08, 0.52, 1],
 [ 0.08, 0.00, 0.52, 1],
 [ 0.08,-0.08, 0.52, 1]])
pts_wld[:,0] = -pts_wld[:,0]

R,t = rigid_transform_3D(pts_cam[:,:3],pts_wld[:,:3])
R2 = np.zeros([4,4])
R2[:3,:3] = R
R2[:3,3] = t
R2[-1,-1] = 1

print(R)
print(t)
print(R2)
print(R2.reshape(-1).tolist())

R_cam2wld = R2
pts_res = R_cam2wld.dot(pts_cam.transpose()).transpose()

for i in range(len(pts_cam)):
	plt.plot(pts_cam[i,0],pts_cam[i,1],color='r')
	plt.text(pts_cam[i,0],pts_cam[i,1],str(i),color='r')
	plt.plot(pts_wld[i,0],pts_wld[i,1],color='b')
	plt.text(pts_wld[i,0],pts_wld[i,1],str(i),color='b')
	plt.plot(pts_res[i,0],pts_res[i,1],color='g')
	plt.text(pts_res[i,0],pts_res[i,1],str(i),color='g')


plt.axis('equal')
plt.show()