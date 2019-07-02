# Log Analysis
This folder contains tools making it simple to read information from training and evaluation logs.

The main part of this project are [Jupyter](https://jupyter.org/) notebooks - think of it as a text editor document enriched with executable code.

## What you need to know to use this

If you just want to have a look, GitHub has a viewer for the notebooks, just click into them and enjoy.

For working with the notebooks you need to be familiar with Python code, but the whole process is reasonably simple. Getting to know pandas and matplotlib will help you evolve from the solutions provided to your own bespoke analysis toolkit.

Tinkering and trying things out is highly desirable. Please share your ideas

## Notebooks

There are currently following notebooks:
* `DeepRacer Log Analysis.ipynb` - original notebook provided by the AWS folks (it has things not used in notebooks listed below)
* `Training_analysis.ipynb` - built on top of the first one with some things removed and many added, prepared to monitor the training progress
* `Evaluation_analysis.ipynb` - built on top of the first one, prepared to analyse evaluation data

## Running the notebooks

I recommend setting up a venv for this:
```
python3 -m venv venv
```
(I recommend folder venv as I have already added it to .gitignore)
Then activate:
```
source venv/bin/activate
```
Then install dependencies:
```
pip install shapely matplotlib pandas sklearn boto3 awscli jupyter
```
Then run
```
jupyter notebook
```
From the opened page you can select a notebook to work with.

## Useful hints
* logs and reward folders have been configured to be ignored by git. This is so that you don't accidentally submit your reward functions or other useful info. Just make sure you secure it somehow yourself.
* have a look at new_reward function usage in the notebooks. It lets you try and evaluate what the reward would look like for a different reward function.

## What can I contribute?

There is a number of opportunities for improvement:
* Report issues/feature requests
* Fix things
* Improve descriptions
* Provide more resources
* Add analysis bits to notebooks
* Complete the `logs_to_params` method in log_analysis to improve the logs replay for a different reward
* Fill in track data used in breakdown in `Training_analysis.ipynb`
* Make the notebooks work with more tracks
