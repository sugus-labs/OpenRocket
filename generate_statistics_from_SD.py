import fileinput
import collections
import time

file_path = '/media/ABB4-4F3A/DATALOG.TXT'

def split_in_blocks(txt_file, pattern):
	'''
	Find the last appears of the text that indicate a new flight and divide in the number of blocks generated by the rocket
	Return: A list that contains all the different blocks of data and a list containing the header.
	'''
	num_times_find_pattern = []
	for num_line, line in enumerate(fileinput.input(txt_file)):
		if pattern in line:
			num_times_find_pattern.append(num_line)
		if num_line == 0:
			header = list(line.strip().split(","))
			#print header
	blocks_of_data = []
	with open(txt_file) as f:
		lines = f.readlines()
		for num_header_line in num_times_find_pattern: 
			if num_header_line == 0:
				num_header_line_prev = num_header_line
			else:
				block_lines = lines[num_header_line_prev + 1 : num_header_line - 1]
				blocks_of_data.append(block_lines)
				num_header_line_prev = num_header_line
		block_lines = lines[num_header_line_prev + 1 : num_line + 1]
		blocks_of_data.append(block_lines)
	return blocks_of_data, header

def manage_data_from_blocks(blocks, header):
	'''
	Divide al the text in blocks tagged with their type of data (accelaration, temperature, ...) continued by a number of block
	Return: A dict that contains all the different types of data diferentiated and numbered.
	'''
	blocks_dict = collections.OrderedDict()
	for block_number, block in enumerate(blocks):
		for item in header:
			blocks_dict['%s%s' % (item,block_number)] = []
		for line in block:
			line_list = line.strip().split(",")
			blocks_dict['f%s' % block_number].append(int(line_list[0]))
			blocks_dict['ax%s' % block_number].append(float(line_list[1]))
			blocks_dict['ay%s' % block_number].append(float(line_list[2]))
			blocks_dict['az%s' % block_number].append(float(line_list[3]))
			blocks_dict['gx%s' % block_number].append(float(line_list[4]))
			blocks_dict['gy%s' % block_number].append(float(line_list[5]))
			blocks_dict['gz%s' % block_number].append(float(line_list[6]))
			blocks_dict['mx%s' % block_number].append(float(line_list[7]))
			blocks_dict['my%s' % block_number].append(float(line_list[8]))
			blocks_dict['mz%s' % block_number].append(float(line_list[9]))
			blocks_dict['t%s' % block_number].append(float(line_list[10]))
			blocks_dict['p%s' % block_number].append(int(line_list[11]))
			blocks_dict['h%s' % block_number].append(float(line_list[12]))
	#print blocks_dict.keys()
	return blocks_dict

def process_data(blocks_dict):
	fingerprints = []
	temperatures = []
	for block in blocks_dict:
		if block.startswith('f'):
			fingerprints.append(block)
		if block.startswith('t'):
			temperatures.append(block)
	fingerprint_basic_info = basic_process_fingerprints(fingerprints)
	temp_basic_info = basic_process_temperatures(temperatures)
	#print fingerprint_basic_info
	#print temperatures_basic_info

def basic_process_fingerprints(fingerprints):
	fingerprint_basic_info = collections.OrderedDict()
	for fingerprint_block in fingerprints:
		millis_interval_list = []	
		for position, millis in enumerate(blocks_dict[fingerprint_block]):
		 	if position != 0:
		 		millis_interval = millis - millis_prev
		 		millis_interval_list.append(millis_interval)
		 	millis_prev = millis
		fingerprint_basic_info[fingerprint_block] = {}
		millis_average = sum(millis_interval_list) / len(blocks_dict[fingerprint_block])
		fingerprint_basic_info[fingerprint_block]["Av"] = millis_average
		millis_max = max(millis_interval_list)
		fingerprint_basic_info[fingerprint_block]["MAX"] = millis_max
		millis_min = min(millis_interval_list)
		fingerprint_basic_info[fingerprint_block]["MIN"] = millis_min
	return fingerprint_basic_info

def basic_process_temperatures(temperatures):
	temp_basic_info = collections.OrderedDict()
	for temp_block in temperatures:
		temp_basic_info[temp_block] = {}
		temp_average = sum(blocks_dict[temp_block]) / len(blocks_dict[temp_block])
		temp_basic_info[temp_block]["Av"] = "%.2f" % temp_average 
		temp_max = max(blocks_dict[temp_block])
		temp_basic_info[temp_block]["MAX"] = temp_max
		temp_min = min(blocks_dict[temp_block])
		temp_basic_info[temp_block]["MIN"] = temp_min
	return temp_basic_info		

#start = time.time()
blocks, header = split_in_blocks(file_path, "m")
blocks_dict = manage_data_from_blocks(blocks, header)
process_data(blocks_dict)

#stop = time.time()
#total_time = stop -start
#print total_time