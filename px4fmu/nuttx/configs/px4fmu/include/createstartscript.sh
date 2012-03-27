#!/bin/sh

# Create a working directory

# Environmental stuff

wd=`pwd`
workingdir=$wd/img
rcsfile=rcS
rcstemplate=$rcsfile.template
romfsimg=romfs.img
headerfile=nsh_romfsimg.h
uinitscript="init.d/rcS"

rm -rf $workingdir || { echo "Failed to remove the old $workingdir"; exit 1; }
mkdir -p $workingdir || { echo "Failed to created the new $workingdir"; exit 1; }

# Create the rcS file from the rcS.template

if [ ! -r $rcstemplate ]; then
    echo "$rcstemplete does not exist"
    rmdir $workingdir
    exit 1
fi

cat $rcstemplate | \
    sed -e "s,XXXMKRDMINORXXX,$fatdevno,g" | \
    sed -e "s,XXMKRDSECTORSIZEXXX,$fatsectsize,g" | \
    sed -e "s,XXMKRDBLOCKSXXX,$fatnsectors,g" | \
    sed -e "s,XXXRDMOUNTPOUNTXXX,$fatmpt,g" >$rcsfile

# And install it at the specified relative location
mkdir -p $workingdir/init.d/
install -m 0755 $rcsfile $workingdir/$uinitscript || \
    { echo "Failed to install $rcsfile at $workingdir/$uinitscript"; rm -f $rcsfile; exit 1; }
rm -f $rcsfile

# Now we are ready to make the ROMFS image

genromfs -f $romfsimg -d $workingdir -V "NSHInitVol" || { echo "genromfs failed" ; exit 1 ; }
rm -rf $workingdir || { echo "Failed to remove the old $workingdir"; exit 1; }

# And, finally, create the header file

xxd -i $romfsimg >$headerfile || { echo "xxd of $< failed" ; rm -f $romfsimg; exit 1 ; }
rm -f $romfsimg
