#!/bin/bash

# Regular Expression used to ensure the calibration
# variables are floating point numbers
re="^-?[0-9]+([.][0-9]+)$"

# First, make sure the calibration config file exists
if [[ -f "../../config/calib.config" ]] ; then
    echo "Sourcing calibration data"
    source ../../config/calib.config

    # Second, make sure that the BIAS and SCALE variables exist
    if [[ $BIAS && $SCALE ]] ; then

        # Third, make sure that these variables are valid numbers (using $re)
        if [[ $BIAS =~ $re  && $SCALE =~ $re ]] ; then
            echo "Found the correct variables, redirecting to firmware header..."

            # If the file exists, remove it
            if [[ -f "../../firmware/calib.h" ]] ; then
                rm ../../firmware/calib.h
            fi

# FILE TEMPLATE TO BE WRITTEN TO THE HEADER FILE ------------------------------
###############################################################################
            cat << EOM > ../../firmware/calib.h
#ifndef _CALIB_H_
#define _CALIB_H_

/**
 * FILE AUTOMATICALLY GENERATED ON BUILD
 * STORES THE CALIBRATION RESULTS
 */

#define BIAS ($BIAS)
#define SCALE ($SCALE)

#endif

EOM
###############################################################################

            echo "Done"
        else
            echo "Please ensure that the variables are floating point numbers"
        fi

    else
        echo "Either BIAS or SCALE was not set. Aborting."
    fi

else
    echo "Could not find calibration file"
fi
