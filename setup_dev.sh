#!/bin/bash

dockerfile="docker-dev.yml"

echo "Killing old development containers..."
docker stop $(docker ps -a -q)

echo "Building containers..."
docker compose -f $dockerfile build

echo "Starting containers..."
docker compose -f $dockerfile --env-file .env.dev up -d

sleep 0.5

echo "DEBUG: Quads Docker Container Up!"
echo "Run docker exec in order to go into the Docker container"
