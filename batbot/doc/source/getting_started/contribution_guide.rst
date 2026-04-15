.. _contribution_guide:

Contribution Guide
====================

Contributing Code
###################

If you're adding new code features, please consider following the workflow below:

1. Write your code
    After setting up the project you can get to writing new code. When working on a new feature to add to the code, please create a new local branch off of ``dev`` dwith a descriptive name.
    When you have made progress on your code, push your local branch to the github repository.

2. Write Tests
    Refer to the :ref:`testing-guide` for information on how to test your code.

3. Write documentation
    Please write detailed documentation for any new classes, functions, etc. you create.

4. Submit a pull request
    After completing, testing, and documenting your code, submit a pull request with your finalized changes.

    Ensure your code passess all CI checks before submitting your pull request. You can check the status of your commits by checking the Actions tab in the Github web browser.

    After resolving all issues, request review from the github administrator, as this will be required for merging your code into the main code base.

Writing Documentation
#######################

Another way to contribute is to write documentation for existing code. Documentation for this project is located in the ``doc`` directory and is written with reStructuredText.
Sphinx is then used to generate these pages from the reStructuredText files.
The following resources can help familiarize you with writing documentation with reStructuredText:

- https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html
- https://docutils.sourceforge.io/docs/user/rst/quickref.html

Previewing Documentation
^^^^^^^^^^^^^^^^^^^^^^^^^^
The official documentation is located at https://bist-research.github.io/, but this documentation reflects the most recent changes on the `release` branch.
Any changes made to the documentation and code can be previewed on your local machine by running:

::

    make html

inside the ``doc`` folder. If you made any changes to the Python code docstrings, make sure to re-install the Python code to see the documentation changes reflected, i.e. run:

::

    pip install .

inside of the ``batbot_bringup`` folder.

For Administrators
######################

There are two important branches in this repository:

- `release`: This branch contains the most up-to-date working code and you should point any researchers in the lab to utilize this branch for their work. 
- `dev`: This branch is used for integration of all new features. Any new work should be merged into this branch first to ensure that everything works together correctly.

These branches are protected, so other team members are not allowed to push commits without first submitting a pull request, passing CI tests, and getting your approval.
Administrators are allowed to override these checks.
The general workflow we've been following is:

1. Create a new feature branch off of `dev` to work on new code
2. After the code is complete, merge it into `dev` after all checks and tests pass
3. Merge `dev` into `release` and implement any finalizing changes. Merging into `release` also automatically uploads the repo documentation.
4. Merge `release` back into `dev` if any changes were made to `release`.

As alluded to above, there are checks and automatic actions that occur during this workflow. 
These are achieved via GitHub actions.
Some actions such as deploying documentation and running testing servers rely on GitHub secrets for authentication with certain tasks.
If any actions are ever failing unexpectedly check the logs as some of these secrets may have expired 
(e.g the personal-access token being used to update the documentation is currently set to expire on Jan 1, 2026).