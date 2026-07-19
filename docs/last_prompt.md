# The last prompt executed in the Cursor Agent window:

## BEGIN: 2026-07-18 19:03 -0700
For the v3 project, define separate implementation phases to satisfy the requirements in @spark_isaac_mycobot_v3/spec.md .

If Residual RL makes sense in this context create an additional phase for this work.

Also define phase(s) for testing on a physical MyCobot 280 M5 arm.

Obtain the resources needed for IsaacSim simulation, copying from the V2 project if applicable.
## END

# Old prompts:

## BEGIN: 2026-07-18 18:45 -0700
I am going to create a new version of this project, located in the spark_isaac_mycobot_v3 directory.

Notice the existence of an existing spec.md and @spark_isaac_mycobot_v3/.cursor directory (equivalent to .cursorrules) in this directory.

Perform the following steps to bootstrap this project:

- Determine the files that would be most useful for a new version of this project. The new project will have different requirements and a different design.

- Copy these file into the new project directory @/workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v3/. If the same or related files exist in the new project directory consider the content of the new files to be primary and use it to replace any conflicting content you find in the files from the previous version.

- Use your own judegement when determining what content from the previous version to omit. If you find certain content would be useful with modifications, then perform those modifications and document appropriately.

- Project documents that relate to current status should be created new and initialized for this new version.

- Push this project to github in a wip_phase0 branch. Leave the main branch empty for now, since it will be reserved for releases.
## END
