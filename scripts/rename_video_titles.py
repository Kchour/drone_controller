"""get video metadata and rename current files according to timestamps


Warning: This will rename files in your target directory (not nested though)


usage: rename_timestamps.py [-h] [-p PATH]

Rename videos in target location according to creation date

optional arguments:
  -h, --help            show this help message and exit
  -p PATH, --path PATH  target directory

Requires tinytag and hachoir packages!

"""
import os
import pdb
import argparse
import re
from tinytag import TinyTag

from hachoir.parser import createParser
from hachoir.metadata import extractMetadata

def get_video_metadata(path):
   """
       Given a path, returns a dictionary of the video's metadata, as parsed by hachoir.
       Keys vary by exact filetype, but for an MP4 file on my machine,
       I get the following keys (inside of "Common" subdict):
           "Duration", "Image width", "Image height", "Creation date",
           "Last modification", "MIME type", "Endianness"

       Dict is nested - common keys are inside of a subdict "Common",
       which will always exist, but some keys *may* be inside of
       video/audio specific stream subdicts, named "Video Stream #1"
       or "Audio Stream #1", etc. Not all formats result in this
       separation.

       :param path: str path to video file
       :return: dict of video metadata
   """

   if not os.path.exists(path):
       raise ValueError("Provided path to video ({}) does not exist".format(path))

   parser = createParser(path)
   if not parser:
       raise RuntimeError("Unable to get metadata from video file")

   with parser:
       metadata = extractMetadata(parser)

       if not metadata:
           raise RuntimeError("Unable to get metadata from video file")

   metadata_dict = {}
   line_matcher = re.compile("-\s(?P<key>.+):\s(?P<value>.+)")
   group_key = None  # group_key stores which group we're currently in for nesting subkeys
   for line in metadata.exportPlaintext():  # this is what hachoir offers for dumping readable information
       parts = line_matcher.match(line)  #
       if not parts:  # not all lines have metadata - at least one is a header
           if line == "Metadata:":  # if it's the generic header, set it to "Common: to match items with multiple streams, so there's always a Common key
               group_key = "Common"
           else:
               group_key = line[:-1]  # strip off the trailing colon of the group header and set it to be the current group we add other keys into
           metadata_dict[group_key] = {}  # initialize the group
           continue

       if group_key:  # if we're inside of a group, then nest this key inside it
           metadata_dict[group_key][parts.group("key")] = parts.group("value")
       else:  # otherwise, put it in the root of the dict
           metadata_dict[parts.group("key")] = parts.group("value")

   return metadata_dict

# command line interface
parser = argparse.ArgumentParser(description="Rename videos in target location according to creation date")
parser.add_argument('-p', '--path', help="target directory")
args = vars(parser.parse_args())


# get user path
# home = expanduser("~")
# get current working directory
# target_dir = os.getcwd()
target_dir = args["path"]

# get all files within directory

for (dirpath, dirnames, filenames) in os.walk(target_dir):
    for f in filenames:
        # only look for mp4 videos
        if "MP4"  in f or "mp4" in f:
            path = os.path.join(dirpath, f)
            # print(os.path.join(dirpath, f))
            # get creation time data from metadata
            # md = get_video_metadata(path)
            # creation_date = md["Common"]["Creation date"]
            pdb.set_trace()
            md = TinyTag.get(path)
            # get metadata time info
            md_time = md.year
            # proc the names 
            _temp = md_time.split("T")
            _date, _time = _temp
            _date = _date.replace("-", "")
            _time = _time.split("-")[0].replace(":","")
            #merge
            new_name = "".join([_date,"-", _time])
            
            # rename current file
            ## get file extension
            ext = f.split('.')[1]
            ## new name
            new_name = "".join([new_name, '.',  ext])
            new_path = os.path.join(dirpath, new_name)
            ## rename operation
            print("renaming {} to {}".format(path, new_path))
            os.rename(path, new_path)