#!/usr/bin/python
import sys

if __name__ == '__main__':
	argv = sys.argv
	n = int(argv[1])

	try:
		print('Creating swarm file for ' + argv[1] + ' individuals...')
		file = open('s' + argv[1] + '.launch','w')

		file.write("<?xml version=\"1.0\"?>\n")
		file.write("<!-- Swarm made of " + argv[1] + " quadrotors -->\n\n")
		file.write("<launch>\n\n")

		file.write("\t<!-- Initial positions (editable) -->\n")
		for i in range(n):
			file.write("\t<arg name=\"x" + str(i) + "\" default=\"0.0\"/> <!-- Edit -->\n")
			file.write("\t<arg name=\"y" + str(i) + "\" default=\"0.0\"/> <!-- Edit -->\n")
		
		file.write("\n\t<!-- Spawn quadrotors in positions -->\n")
		for i in range(n):
			file.write("\t<include file=\"$(find swarm)/launch/call_quad.launch\">\n")
			file.write("\t\t<arg name=\"name\" value=\"uav" + str(i) + "\"/>\n")
			file.write("\t\t<arg name=\"x\" value=\"$(arg x" + str(i) + ")\"/>\n")
			file.write("\t\t<arg name=\"y\" value=\"$(arg y" + str(i) + ")\"/>\n")
			file.write("\t</include>\n")
		
		file.write("\n\t<!-- Export positions as parameters -->\n")
		for i in range(n):
			file.write("\t<group ns=\"uav" + str(i) + "\">\n")
			file.write("\t\t<param name=\"x\" value=\"$(arg x" + str(i) + ")\" type=\"double\"/>\n")
			file.write("\t\t<param name=\"y\" value=\"$(arg y" + str(i) + ")\" type=\"double\"/>\n")
			file.write("\t</group>\n")
		
		file.write("\n</launch>\n")
		file.close()
		print('Done!')

	except:
		print('Something went wrong!')
		sys.exit(0) # quit Python
