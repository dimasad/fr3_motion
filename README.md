# fr3_motion
Controllers and utilities for moving Franka Research 3 robots using ROS2.

Development Container
=====================
The packages in this workspace implement controllers and utilities for moving
FR3 robots from Franka using ROS. The controllers can be developed and deployed
using the container and compose file at the repository root. The container
described in the Dockerfile at the repository root is built automatically
using GitHub Actions and pushed to the GitHub Container Registry 
<https://ghcr.io/dimasad/fr3_motion>.

To start the container for development, run the following at the repository root.

```
docker compose up -d
```

Then, to open an interactive terminal on the container, run the following.
You may use this command to open multiple terminals, as typical in ROS
workflows. The repository root is mapped to `/workspace/` inside the
container and can be used for development.

```
docker compose exec dev bash
```

After you are done, you may tear down the container with the command below,
which removes the container instance. All files outside `/workspace` and any
changes made to the container will be lost.

```
docker compose down
```
