# orchestration-lab

Playground for orchestration related services especially in a k3d/k3s environment.

## Prerequisites

If not available already, install the following:

- [Docker](https://docs.docker.com/engine/install/ubuntu/) 
- [K3D](https://k3d.io/v5.6.0/#installation)
- [kubectl](https://kubernetes.io/docs/tasks/tools/#kubectl)

## General Structure

- [pod-expose](pod-expose) tests all the ways to expose your cluster to the outside in k3d, except load balancing and ingress rules which are done in [load-balancing in k3d](load-balancing)
- [load-balancing in k3d](load-balancing) just tests how Traefik works in k3d and how you can use it to host a minimal web setup to allow traffic inside your cluster.
- just testing customization in Kubernetes, because big fan of customizing in Linux (Check out [nix-dotfiles](https://github.com/UmutAlpTuglu/nix-dotfiles)), by changing a crucial part of the k3s architecture. K2s uses Flannel CNI which runs faster than Calico, which is normally run in k8s archietctures. In [custom CNI folder](custom/CNI) I just remove the old CNI and add a new one in a minimal way.
- app structure and pipelines are described below.

## App Structure

So far there are two apps with two pipelines developed in this repo. The [py.dockerfile](py.dockerfile) can be used via `Dev Containers` Extensions in `vscode` and is for the microservices folder, where you can find exemplaraly a nasa application. All new python dependencies (libs) can be added via [requirements.txt](requirements.txt). 

### Nasa App

**Prerequisites**

- create docker hub with read - write access token
- create Nasa API key
- have a server to deploy to
- create secrets for actions as mentioned in the pipeline (also add Nasa API key)

Here you can find the [details](microservices/nasa/README.md) of the app.
For the [nasa app](microservices/nasa) there is a [CI/CD pipeline](.github/workflows/nasa_deploy.yml) deployed where images get sent to a private docker hub registry and then are pulled to a server and the docker container is automatically run in detached mode on a server and saves the image in a designated folder (path can be altered in pipeline). 

The special thing about this pipeline is that it gets triggered via git tags, which also get used in all the subsequent jobs.
This is how to trigger it from your host:
```shell
git tag v1.0.1
git push origin v1.0.1
```

### Pointcloud ROS app

Inside `ws` there is an entire python ros 2 development environment via docker and debug environment, open the ws folder with vscode and then trigger the `Dev Containers` folder. 
In the [ros package folder](ws/src/pointcloud_detection) you can find the entire package setup and how to test the code can be found as comments at the top of this [file](ws/src/pointcloud_detection/pointcloud_detection/pointcloud_detection.py). You can test the application in the docker development environment with three terminals, or you can build the [pointcloud.Dockerfile image](ws/pointcloud.Dockerfile) locally and run it or you can use the [CI ros build pipeline](.github/workflows/ros_build.yml), which just builds the package and pushes it to a public docker hub image registry. Its triggered when one of the dependent files is changed and there is a push to the main branch. 
