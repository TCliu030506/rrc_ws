<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>inspire_pkg</name>
  <version>0.0.1</version>
  <description>Metapackage that groups several packages</description>
  <maintainer email="3150103428@zju.edu.cn">rus</maintainer>
  <license>BSD-3-Clause</license>

  <!-- Dependencies -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <!-- Add any buildtool dependencies as needed -->

  <!-- Metapackage dependencies -->
  <depend>inspire_msg</depend>
  <depend>inspire_actuator</depend>
  <depend>insactuator_client</depend>
  <!-- Add more dependencies as needed -->

  <!-- Additional custom tags can be added here if needed -->
</package>