=================
realsense-capture
=================


Realsense-Capture package


Description
===========

A package to simply detect and connect to the Realsense camera.

All steps of Creating and Developing projects
=============================================

Preparing::
***********

    putup -i realsense-capture
    cd realsense-capture
    pip install -e . # install dependencies
    git init --initial-branch=main
    git remote add origin https://...

    pre-commit install
    pre-commit autoupdate
    
Checking::
**********
    
    tox # unit tests
    pre-commit run --all-files # check conventions

Push modification::
*******************

    git tag v0.1.0 # assign version
    tox -e build # build packages
    tox -e docs # build documents

    git add .
    git push -u origin main




.. _pyscaffold-notes:

Making Changes & Contributing
=============================

This project uses `pre-commit`_, please make sure to install it before making any
changes::

    pip install pre-commit
    cd realsense-capture
    pre-commit install

It is a good idea to update the hooks to the latest version::

    pre-commit autoupdate

Don't forget to tell your contributors to also install and use pre-commit.

.. _pre-commit: http://pre-commit.com/

Note
====

This project has been set up using PyScaffold 4.0.2. For details and usage
information on PyScaffold see https://pyscaffold.org/.
