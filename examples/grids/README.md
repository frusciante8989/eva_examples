# Grid Example

This example code shows how a user might go about implementing their own 2D grid system. It provides a 2D grid class, which provides a series of grod positions with input of:

- 3 XY corner positions
- A number of rows
- A number of columns

Using this 2D grid as a building block, you will be able to create more complex interactions, for example picking from one grid and placing into another!

## Compatibility

Requirements to run this script as is:

- Eva software version: 3.x.x
- Eva Python SDK: 2.x.x

## Setting up and running

This project has used pipenv for dependency mamangement, please ensure you have Python3 and pipenv installed.

    # Download the example code and enter navigate the terminal to this directory
    $ pipenv install

    # Caution, this will cause Eva to move!
    $ pipenv run main.py