__author__ = 'glalonde'

import xml.etree.ElementTree as ET
import fnmatch
import os

'''
Stupidly generic parser for xml defined anythings
Just make a parsing method (.from_xml_node()) in the main class.

'''

class XMLDefinedLibrary:
    def __init__(self, library_directory):
        self.library = {}
        self.parse_folder(library_directory)

    def parse_folder(self, path):
        xml_files = []
        for root, dirnames, filenames in os.walk(path):
            for filename in fnmatch.filter(filenames, '*.xml'):
                xml_files.append(os.path.join(root, filename))

        num_to_parse = len(xml_files)
        done = num_to_parse > 0
        while not done:
            failed = []
            for file in xml_files:
                try:
                    self.generic_parse(file)
                except:
                    continue


    def generic_parse(self, path):
        tree = ET.parse(path)
        root = tree.getroot()

        try:
            type = root.get('type')
            project_path = root.get('path')
        except:
            raise ValueError('XML definition is missing \'type\' and/or \'path\' definitions in top level node')


        XMLDefinedLibrary.generic_import(type, project_path)


        if type not in self.library:
            self.library[type] = {}

        for child in root:
            # Create the new type
            metatype = child.get('name')
            try:
                new_object = eval('%s.from_xml_node(child)') % (type)
            except:
                raise ValueError('XML parser failed for type %s : %s' % (type, metatype))
            self.library[type][metatype] = new_object

    @staticmethod
    def generic_import(type, project_path):
        eval_string = 'from %s import %s' % (project_path, type)
        exec(eval_string)
        try:
            exec(eval_string)
        except:
            raise ValueError('Import failed for type; Are the project path and type correct?: %s' % eval_string)

lib = XMLDefinedLibrary('../configurations-new/')
