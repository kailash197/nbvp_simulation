```bash
cd ~/nvbp-docker && docker compose -f docker-compose.yml build
xhost +local:docker
cd ~/nvbp-docker && docker compose -f docker-compose.yml run --remove-orphans nbvp
```

```bash
sudo apt-get update && sudo apt-get install -y docker-compose
```

```bash
cd ~/nvbp-docker && docker compose -f docker-compose.yml build uav_ros_sim
xhost +local:docker
cd ~/nvbp-docker && docker compose -f docker-compose.yml run --remove-orphans uav_ros_sim
```