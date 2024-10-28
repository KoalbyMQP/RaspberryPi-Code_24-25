from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
left_arm_chain = Chain(name='left_arm', links=[
    OriginLink(),
    URDFLink(
      name="shoulder",
      origin_translation=[-10, 0, 5],
      origin_orientation=[0, 1.57, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="elbow",
      origin_translation=[25, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="wrist",
      origin_translation=[22, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    )
])
# Set the active links mask (ensure the base link is inactive)
left_arm_chain.active_links_mask = [False, True, True, True]
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
left_arm_chain.plot(left_arm_chain.inverse_kinematics([2, 10, 10]), ax)
matplotlib.pyplot.show()