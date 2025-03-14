
# python-template

Template for python projects
Mainly based on [goodresearch ](https://goodresearch.dev).

## Usage
Just run the following:
```
setup.sh [NEW_NAME]
```
This is going to replace the template {{python_template}} with the desired new library name, and the respective placeholders in the setup tools. 
The default pixi install considers the basic of pytorch and cuda12.0. This is quite specific so going to think to add more default solutions.


## Defines the structure of each project. 
```
| -- configs
| -- data
| -- logs
| -- results
| -- scripts
| -- tests
-- .gitignore
-- enviroment.yml
-- README.md
```

- **data:** where to large files such as datasets, results and others
- **logs:** Folder that works as a dump for different logs of experiments
- **results:** Nicer version of the visualization of what was saved in each of the logs, here results should be in the form of folder or in some case jupyter-notebooks that explain the experiments, by default this folder is not syncqed with github unless is 
- **Configs:** Different configurations to run the experiments, also config files for different robot configurations
- **scripts:** All the files that are self contained that usually only perform one thing, then can be changed using arguments
- **tests:** Scripts that test a functionality an in some cases the interactive scripts that allows to perform differnet modifications

