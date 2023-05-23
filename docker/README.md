# Train yolov5 on huggingface dataset
## download dataset
https://huggingface.co/datasets/St333fan/StrawberryRedGreenFlower
```
# Make sure you have git-lfs installed (https://git-lfs.com)
git lfs install
mkdir resources
cd resources
git clone https://huggingface.co/datasets/St333fan/StrawberryRedGreenFlower
```

## check 
```
sudo docker build -t <imageName> .
sudo docker run -dit -v ./dataset:/app/dataset <imageName>
sudo docker exec -it <containerId> /bin/bash
```

## training
```
docker compose up --build
```

## copy results
```
docker cp <containerId>:/app/yolov5 ./results
```