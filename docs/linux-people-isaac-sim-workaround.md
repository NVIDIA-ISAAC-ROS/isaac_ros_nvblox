# Workaround for running People Animation on Linux

Currently, python scripts for animation can only run if the usd is on the same filesystem as the script. This means that it is required to upload the scripts next to where the usd is stored:

1. To retrieve where the scripts are stored, click Window -> Extensions and search for `omni.anim.people`. Then in the extension details click on the folder icon. This will open a window in your browser showing where the extension is stored.

<div align="center"><img src="../resources/isaac_sim_anim_people_scripts.png" width=700px/></div>

2. You now need to upload the `scripts` folder from the `omni/anim/people` folder into a `people_anim_scripts` folder next to the scene usd. To do so, you can right-click on the *Content* tab on the bottom then select *Upload files and folders* to upload the scripts folder.

3. This step will need to be done each time a new version of the extension is released.
