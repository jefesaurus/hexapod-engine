__author__ = 'glalonde'


class XmlDefinable:
    @classmethod
    def from_xml_node(cls, node):
        raise NotImplementedError('XML parsing has not been implemented for this class yet!')