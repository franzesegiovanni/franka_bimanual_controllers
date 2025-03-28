# %%
from panda import Panda
import rospy
#%%
rospy.init_node("learning_node")
panda_left=Panda(arm_id='panda_left')
panda_right=Panda(arm_id='panda_right')
#%%
panda_left.traj_rec()
#%%
panda_left.save(name="test")
#%%
panda_left.load(name="test")
#%%
panda_left.execute(offset_z=0,  global_frame=True)
#%%
panda_left.home()

# %%
panda_right.load(name="test")
#%%
panda_right.execute(offset_z=-0.015, global_frame=True)
#%%
panda_right.home()



# %%
