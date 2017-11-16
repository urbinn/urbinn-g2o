## A start for developing g2o python bindings for the Urbinn project

## Developing

```git clone --recursive git://github.com/jvhoven/urbinn-g2o.git```

We depend on a specific version of g2o, therefore it is bundled with the repository.
The development is currently done within Docker to reflect our production environment,
in order to set this up you'll have to run the following commands:

### Through Docker

```./run-docker.sh```

This will setup the required docker container based on Ubuntu 17.10, install the required packages
and build g2o. *This step can take a while*.

In order to test the implementation you can run the following command:

```docker run urbinn/g2o```

### Locally

You can attempt to run it locally by calling `bootstrap.sh` but the results may vary depending on your system.

```
./bootstrap.sh

Available arguments
--server - sets the csparse include directory to `/opt/jupyterhub/anaconda/include`
```

---

### Improvements

- [ ] **Figure out a better Docker development workflow** <br />Instead of rebuilding the container being able to push to a running container for example. Not sure how Docker development workflows are.

- [ ] **Filter out build directories when trying to setup a locally build project in Docker.**

