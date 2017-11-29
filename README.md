## A start for developing g2o python bindings for the Urbinn project

## Developing

```git clone --recursive git://github.com/jvhoven/urbinn-g2o.git```

We depend on a specific version of g2o, therefore it is bundled with the repository.
The development is currently done within Docker to reflect our production environment,
in order to set this up you'll have to run the following commands:

### Through Docker

```./scripts/run-docker.sh <directory/to/local/project/folder>```

This will setup the required docker container based on Ubuntu 17.10, install the required packages
and build g2o. *This step can take a while*.

### Locally

You can attempt to run it locally by calling `bootstrap.sh` but the results may vary depending on your system.

```
./scripts/bootstrap.sh

Available arguments
--server - sets the csparse include directory to `/opt/jupyterhub/anaconda/include`
```

### Testing

Testing is as simple as invoking:

```
python3 setup.py test
```

### Installing

For checking if the package would work without globally installing it:

```
python3 setup.py develop
```

For globally installing the package:

```
python3 setup.py install
```

## Usage

There is currently no documentation available. For more information on how to use certain bindings of this project
refer to the `tests` directory.





