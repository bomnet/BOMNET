import os.path
import sys
import ifcopenshell
import numpy as np
from ifcopenshell import geom
from ifcopenshell.util import shape, placement
import xml.dom.minidom as minidom

OMNETPP_INI = ['# GENERATED BY BOMNET\n', '# For a 3D visualization, activate the 3D features in inet4.5 project\n',
               '# then you can build your network (YOUR_NETWORK.ned) as the following example:\n',
               '##########################   BOMNetwork.ned   ##########################\n', '#\n',
               '#  package inet.tutorials.bomnet;\n',
               '#  import inet.environment.common.PhysicalEnvironment;\n',
               '#  import inet.visualizer.common.IntegratedVisualizer;\n', '\n', '#  network BOMNetwork\n', '#  {\n',
               '#      submodules:\n', '#          visualizer: IntegratedVisualizer {\n',
               '#              parameters:\n', '#                  @display("p=100,100");\n', '#          }\n',
               '#          physicalEnvironment: PhysicalEnvironment {\n', '#              @display("p=100,100");\n',
               '#          }\n', '#  }\n', '#\n',
               '########################################################################\n', '\n',
               '[General]\n', 'description = Adding BIM to the environment\n',
               'network = #YOUR_NETWORK or BOMNetwork if you use the example above \n', '\n',
               '*.physicalEnvironment.config = xmldoc("environment.xml")\n',
               '*.radioMedium.obstacleLoss.typename = "DielectricObstacleLoss"\n', '\n', '# ADD HERE YOUR CONFIG']


class ElementParser:
    """
    class that provides utility functions
        Attributes:
            - element (ifcopenshell.entity_instance): IfcObject.

        Methods:
            - get_global_verts(self): returns a global vertices.

    """

    def __init__(self, element):
        if not isinstance(element, ifcopenshell.entity_instance):
            raise ValueError("element must be Ifcopenshell object")
        else:
            settings = ifcopenshell.geom.settings()
            try:
                self.element_shape = ifcopenshell.geom.create_shape(settings, element)
            except Exception as e:
                raise e

        self.element = element

    def get_global_verts(self):
        """
        returns a global vertices
        """

        # shape.get_element_vertices(element,geometry) not working, matrix mismatch in matrix multiplication
        verts = shape.get_vertices(self.element_shape.geometry)
        mat = placement.get_local_placement(self.element.ObjectPlacement)
        g_verts = np.array([(mat @ np.array([*vert, 1]))[:3] for vert in verts])

        return g_verts.tolist()


class Worker:
    def __init__(self, path: str, destination: str):
        self.output_dict = {}
        self.destination = destination

        try:
            self.ifc_file = ifcopenshell.open(path)
        except Exception as e:
            self.ifc_file = None
            print(e)

    def extract(self):

        if self.ifc_file is None:
            return 1

        def get_min_xyz(_vertices):

            min_x = min(_vertices, key=lambda point: point[0])[0]
            min_y = min(_vertices, key=lambda point: point[1])[1]
            min_z = min(_vertices, key=lambda point: point[2])[2]

            return [min_x, min_y, min_z]

        def get_str_of_points(points):
            _str = ""

            for p in points:
                _str += f'{round(p[0], 5)} {round(p[1], 5)} {round(p[2], 5)} '

            return _str

        walls = self.ifc_file.by_type("IfcWall") + self.ifc_file.by_type("IfcWallStandardCase")
        slabs = self.ifc_file.by_type("IfcSlab") + self.ifc_file.by_type("IfcRoof") + self.ifc_file.by_type(
            "IfcFooting")
        windows = self.ifc_file.by_type("IfcWindow")

        plates = self.ifc_file.by_type("IfcPlate")

        doors = self.ifc_file.by_type("IfcDoor")
        stairs = self.ifc_file.by_type("IfcStair") + self.ifc_file.by_type("IfcMember") + self.ifc_file.by_type(
            "IfcStairFlight")
        beams = self.ifc_file.by_type("IfcBeam")
        columns = self.ifc_file.by_type("IfcColumn")

        # logo = self.ifc_file.by_type("IfcBuildingElementProxy")

        ifc_objects = walls + slabs + windows + doors + stairs + beams + columns + plates  # + logo

        idx = 0

        for ifc_object in ifc_objects:

            # default values
            opacity = 1
            color = "128 128 128"
            material = "concrete"

            if ifc_object.is_a() == "IfcSpace":
                continue

            try:
                object_shape = ElementParser(ifc_object)
                vertices = object_shape.get_global_verts()
                faces = shape.get_faces(object_shape.element_shape.geometry)
            except Exception as e:
                print(e)
                continue

            if ifc_object.is_a() == "IfcWall" or ifc_object.is_a() == "IfcWallStandardCase":
                color = "170 170 170"
                opacity = 0.9
            elif ifc_object.is_a() == "IfcSlab" or ifc_object.is_a() == "IfcColumn":
                color = "150 150 150"
            elif ifc_object.is_a() == "IfcDoor" or ifc_object.is_a() == "IfcMember" or ifc_object.is_a() == "IfcStairFlight" or ifc_object.is_a() == "IfcStair":
                opacity = 0.5
                color = "150 105 25"
                material = "wood"
            elif ifc_object.is_a() == "IfcWindow" or ifc_object.is_a() == "IfcPlate":
                opacity = 0.2
                color = "43 250 250"
                material = "glass"
            elif ifc_object.is_a() == "IfcBeam":
                material = "copper"

            elif ifc_object.is_a() == "IfcBuildingElementProxy":
                opacity = 1
                material = "aluminium"
                color = "143 254 9"

            my_key = ifc_object.is_a() + ' ' + ifc_object.GlobalId + " '" + ifc_object.Name+"'"
            self.output_dict[my_key] = {"position": "min " + get_str_of_points([get_min_xyz(vertices)]),
                                     #"name": ifc_object.GlobalId,
                                     "shape": "ifcPolyhedron " + get_str_of_points(vertices),
                                     "faces": get_str_of_points(faces),
                                     "material": material,
                                     "fill-color": color,
                                     "opacity": str(opacity)}
            idx += 1
        return 0

    def generate_environment_xml(self):
        def create_xml(objects_dict, shapes_dict, materials_dict, path):
            doc = minidom.Document()
            root = doc.createElement("environment")
            doc.appendChild(root)

            for key, values in objects_dict.items():
                # add comment
                comment = doc.createComment(str(key))
                root.appendChild(comment)
                object_element = doc.createElement("object")
                root.appendChild(object_element)
                for attr, value in values.items():
                    object_element.setAttribute(attr, value)


            for key, values in shapes_dict.items():
                shape_element = doc.createElement("shape")
                root.appendChild(shape_element)
                shape_element.setAttribute("id", key)
                for attr, value in values.items():
                    shape_element.setAttribute(attr, value)

            for key, values in materials_dict.items():
                material_element = doc.createElement("material")
                root.appendChild(material_element)
                material_element.setAttribute("id", key)
                for attr, value in values.items():
                    material_element.setAttribute(attr, value)

            xml_str = doc.toprettyxml(indent="\t")
            output_path = os.path.join(path, "environment.xml")
            with open(output_path, "w", encoding="utf-8") as f:
                f.write(xml_str)

        create_xml(self.output_dict, {}, {}, self.destination)

        return 0

    def generate_omnet_ini(self):
        omnetpp_ini_path = os.path.join(self.destination, "omnetpp.ini")
        with open(omnetpp_ini_path, 'w') as file:
            file.writelines(OMNETPP_INI)


class Coder:
    def __init__(self, inet_path: str):
        self.inet_path = inet_path
        self.plug_in_path = "/io/univgustaveeiffel/ide/bomnet/handlers/"
        # src_path = os.path.join(self.inet_path, "src")
        self.Makefile_path = os.path.join(inet_path, "src/Makefile")

    def write_code(self):
        try:
            # Check if the Makefile exists and add the new file to the list of sources
            if os.path.exists(self.Makefile_path):
                with open(self.Makefile_path, 'r') as file:
                    lines = file.readlines()
                # check if the line is already added
                if any("$O/inet/common/geometry/shape/polyhedron/IfcPolyhedron.o" in line for line in lines):
                    return True
                with open(self.Makefile_path, 'w') as file:
                    for line in lines:
                        if '$O/inet/common/geometry/shape/Sphere.o' in line:
                            file.write(line)
                            file.write("    $O/inet/common/geometry/shape/polyhedron/IfcPolyhedron.o \\\n")
                        elif "$O/inet/common/geometry/shape/polyhedron/IfcPolyhedron.o" in line:
                            continue
                        else:
                            file.write(line)
            return True
        except Exception as e:
            print(e)
            return False


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: generate_files.py <project_path> <selected_file> <inet_path>")
        sys.exit(1)

    project_path = sys.argv[1]
    selected_file = sys.argv[2]
    inet_path = sys.argv[3]

    code_added = Coder(inet_path).write_code()
    if not code_added:
        print("Error adding INET code")
        sys.exit(1)

    if True:
        print("Code added successfully", "\nExtracting BIM data...")
        worker = Worker(selected_file, project_path)
        extraction = worker.extract()
        if extraction == 0:
            print("BIM data extracted successfully", "\nGenerating files...")
            worker.generate_environment_xml()
            worker.generate_omnet_ini()

    print(f"Simulation files have been created in {project_path}")
