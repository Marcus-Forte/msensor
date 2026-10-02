# Deployment

This folder hols deployment runtime. It's a stripped down docker image deployable to any computer with docker engine.

- From the host machine, build and pish with `docker compose build --push`. Rename image name if needed for different registries.
- Copy the docker-compose.yml to the target. Modify if needed.
- Call `docker compose up`. It will automatically pull from the registry, start the container and restart upon reboots.