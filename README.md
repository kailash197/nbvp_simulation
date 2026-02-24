```bash
cd ~/nvbp-docker && docker compose -f docker-compose.yml build
xhost +local:docker
cd ~/nvbp-docker && docker compose -f docker-compose.yml run --remove-orphans nbvp
```

