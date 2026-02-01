# Git Cheat Sheet

## Clone Repository From GitHub
```
git clone https://github.com/Quadconn/quadconn.git
```

## Pull In Latest Changes Of Current Branch
This will pull in the latest changes into the branch you currently have checked out.
```
git pull
```

## Add Current Changes to Next Commit
This adds all changed files within the current directory and its children.
```
git add .
```
You can also specify manually files to add.
```
git add path/to/file
```

## Commit Changes
```
git commit -m "Place a insightful message here about the changes."
```

## Push Local Branch to Server (If not tracked by server yet)
Replace `local_branch` with the name of the branch you want to push onto the server. 
FYI: `origin` here refers to the remote server on GitHub.
```
git push -u origin local_branch
```

## Push Local Branch Changes to Server (If already tracked by server)
```
git push
```

## View All Branches
```
git branch -a
```

## Checkout a Branch
This will place you into the branch you specify. Replace `desired_branch` with 
the name of the branch you want to checkout.
```
git checkout desired_branch
```

## Create New Branch
Replace `new_branch` with the name of the branch you would like to create.
```
git branch example_branch
```


## Delete a Branch
Replace `branch_to_delete` with the name of the branch you want to delete.
```
git branch -d branch_to_delete
```

## Clean Up No Longer Tracked Branches
This will fetch the latest changes from the server but not yet apply them. 
The `--prune` flag removes any branches no longer tracked by the server. 
Optionally you can run `git pull` after to actually apply the changes into 
your current branch.
```
git fetch --prune
```

## Merge A Branch Into the Current Branch
Ensure you are currently in the branch you want to merge changes from the other branch into. For
example if currently in a branch `current_branch` the following would merge in the changes from 
`branch_to_merge` into `current_branch`. You do this usually to handle merge conflicts manually.
```
git merge branch_to_merge
```

## Common Mistakes

### Don't Checkout Origin Branches Directly
When executing `git branch -a` remote branches will also be shown. For example:
```
remotes/origin/branch1
remotes/origin/branch2
```
To checkout one of these branches simply `git checkout` the branch name without the `remotes/origin/` prefix. 
For example to checkout `branch1` from above run `git checkout branch1`
