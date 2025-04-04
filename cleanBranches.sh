#/bin/bash
git fetch -p
if [ $? -ne 0 ]
then
    printf "\n\U01F92E what? git fetch failed!  are you even in a git repo? cause it don't seem like `pwd` is.\n\n"
    exit 1
fi

git checkout main
if [ $? -ne 0 ]
then
    printf "\n\U01F92E oh my golly geepers gracious! I couldn't checkout main.  I'm just stoppin now.\n\n"
    exit 1
fi

for branch in `git branch -vv | grep ': gone]' | awk '{print  $1}'`
do
    git branch -D $branch 
done
git reset --hard origin/main

printf "\n\U01F4AA\U01F49E\U01F525\U01F332\U01F308\U01F499\U01F49A\U01F49B\U01F49C yayayayay your branches are super now.\n\n"
