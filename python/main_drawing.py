# %%
from panda import Panda
import rospy
#%%
rospy.init_node("learning_node")
panda_left=Panda()
panda_right=Panda()
#%%
panda_left.traj_rec()
#%%
panda_left.save(name="circle")
#%%
panda_left.load(name="circle")
#%%
panda_left.execute(offset_z=-0.015,  global_frame=True)
#%%
panda_left.home()

# %%
panda_right.load(name="circle")
#%%
panda_right.execute(offset_z=-0.015, global_frame=True)
#%%
panda_left.home()


