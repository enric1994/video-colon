# video-colon

![Alt Text](demo.gif)



# Setup
In order to run the code [Docker and Docker-Compose](https://gist.github.com/enric1994/3b5c20ddb2b4033c4498b92a71d909da) are required. 

## 1. Starting the containers
First, run `docker-compose up -d` in the `docker/` folder. It will build a Docker container with Blender installed. This can take up to 1 hour. Grab a coffee

## 2. Synthetic videos and annotations generation with Blender
In order to generate the dataset run: `docker exec blender python generate.py --name test_dataset --size 2`. It will generate 2 sequences of 1000 frames in `datasets/test_dataset/frames` and the annotations in `datasets/test_dataset/masks`. Grab another coffee.
