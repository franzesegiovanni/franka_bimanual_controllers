"""
Authors: Giovanni Franzese and Tim Verburg
Email: g.franzese@tudelft.nl
Cognitive Robotics, TU Delft
This code is part of TERI (TEaching Robots Interactively) project
"""
#%%
from dual_panda import DualPanda
import time
#%%
if __name__ == '__main__':
    BiManualTeaching=DualPanda()
    time.sleep(1)
    BiManualTeaching.Panda_left.home_gripper()
    BiManualTeaching.Panda_right.home_gripper()
    time.sleep(1)

#%%
BiManualTeaching.Panda_left.Kinesthetic_Demonstration()
    
# %%
BiManualTeaching.Panda_left.go_to_start()
# %%
BiManualTeaching.Panda_left.execute_traj()
# %%
BiManualTeaching.Panda_left.save()    
#%%
if not BiManualTeaching.Panda_left.pause:
    BiManualTeaching.Panda_left.execute_traj()
    BiManualTeaching.Panda_left.Kinesthetic_Demonstration(active=True)
#%%
BiManualTeaching.Panda_right.Kinesthetic_Demonstration()
# %%
BiManualTeaching.Panda_right.go_to_start()
# %%    
BiManualTeaching.Panda_right.execute_traj()
# %%
BiManualTeaching.Panda_right.save()
# %%
BiManualTeaching.Panda_left.go_to_start()
BiManualTeaching.Panda_right.go_to_start()
# %%  
BiManualTeaching.Panda_right.execute_traj()
BiManualTeaching.Panda_left.execute_traj()
BiManualTeaching.Kinesthetic_Demonstration_BiManual(active=True)
# %%
BiManualTeaching.Kinesthetic_Demonstration_BiManual(active=True)
# %%
BiManualTeaching.Panda_left.home()
BiManualTeaching.Panda_right.home()
#%%
BiManualTeaching.save()
# %%
BiManualTeaching.load()
# %%e
BiManualTeaching.Panda_right.go_to_start()
BiManualTeaching.Panda_left.go_to_start()
#%%    
BiManualTeaching.execute_dual()
# %%
BiManualTeaching.correction_execute_dual()
# %%
BiManualTeaching.enable_correction = False
# %%
BiManualTeaching.Kinesthetic_Demonstration_BiManual()
#%%
BiManualTeaching.go_to_start()
#%%
BiManualTeaching.execute_dual()