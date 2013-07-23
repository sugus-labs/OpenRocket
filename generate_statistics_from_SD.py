import fileinput
from itertools import islice
import time

file_path = '/media/ABB4-4F3A/DATALOG.TXT'
num_times_find_pattern = []
blocks_of_data = []

def find_pattern_in_txt(txt_file, pattern):
	'''
	Find the last appears of the text that indicate a new flight

	Return: The first new data line and the last line of data.
	'''
	for num_line, line in enumerate(fileinput.input(txt_file)):
		if pattern in line:
			num_times_find_pattern.append(num_line)

	with open(txt_file) as f:
		lines = f.readlines()
	print lines[0:5]


#def split_data_from_txt(txt_file, new_initial_line, last_line):


start = time.time()
find_pattern_in_txt(file_path, "milliseconds")
stop = time.time()
total_time = stop -start
print total_time