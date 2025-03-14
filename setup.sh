if [ $# -eq 0 ]
  then
    echo "No arguments supplied"
    exit
fi

new_name=$1



mv \{\{python_template\}\}/ $new_name/
sed -i -e s/src/$new_name/ setup.py
sed -i -e s/src/$new_name/ pyproject.toml
sed -i -e s/src/$new_name/ pixi.toml


rm setup.sh
git add .
git commit -m 'initial commit'
