import glob
import os

flist = glob.glob(os.path.join("lat_lon_files", "*.txt"))
print("Available txt: ")
for path in flist:
    print(path)
print()

basename_list = input("Enter all files to concatenate: ").split()
output_name = input("Enter name for output_file: ")

for basename in basename_list:
    os.system(f"cat lat_lon_files/{basename}.txt >> lat_lon_files/{output_name}.txt")
