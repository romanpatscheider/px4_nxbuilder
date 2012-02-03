#!/bin/sh
mavlink_0_9_tag=f76145c42d
mavlink_1_0_tag=d6bd668673
pymavlink_tag=7659fec138

# get mavlink-0.9
rm -rf mavlink-0.9
git clone git://github.com/pixhawk/mavlink.git -b master mavlink-0.9
cd mavlink-0.9 && git checkout $mavlink_0_9_tag && rm -rf .git && cd ..

# get mavlink-1.0
rm -rf mavlink-1.0
git clone git://github.com/pixhawk/mavlink.git -b v10release mavlink-1.0
cd mavlink-1.0 && git checkout $mavlink_1_0_tag && rm -rf .git && cd ..

# get pymavlink
rm -rf pymavlink
git clone https://github.com/tridge/pymavlink.git -b master  pymavlink
cd pymavlink && git checkout $pymavlink_tag && rm -rf .git && cd ..

# generate the includes
./pymavlink/generator/mavgen.py --lang=C --wire-protocol=0.9 --output=include/mavlink-0.9 mavlink-0.9/message_definitions/common.xml
./pymavlink/generator/mavgen.py --lang=C --wire-protocol=1.0 --output=include/mavlink-1.0 mavlink-1.0/message_definitions/common.xml

# cleanup
rm -rf mavlink-0.9 mavlink-1.0 pymavlink 
