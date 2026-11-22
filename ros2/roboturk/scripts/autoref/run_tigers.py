"""Run TIGERs 1.7.1 in an isolated working directory with its passive event observer."""

import argparse
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("output", type=Path)
    p.add_argument("--distribution", required=True, type=Path)
    p.add_argument("--classes", required=True, type=Path, help="Compiled EventObserver directory")
    p.add_argument("--java", default="java")
    p.add_argument("--mode", choices=("PASSIVE", "ACTIVE"), default="PASSIVE")
    p.add_argument("--ci-port", type=int, default=10013)
    p.add_argument("--gc-port", type=int, default=10007)
    a = p.parse_args()
    folder = a.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    dist = a.distribution.resolve()
    cp = os.pathsep.join((str(a.classes.resolve()), str(dist / "lib/*")))
    java = shutil.which(a.java)
    if not java:
        p.error("Java executable missing")
    java = str(Path(java).resolve())
    shutil.copytree(dist / "config", folder / "config")
    file = folder / "config/moduli/autoreferee-ci.xml"
    xml = ET.parse(file)
    root = xml.getroot()
    ET.SubElement(root.find("globalConfiguration"), "simulation").text = "true"
    root.find("globalConfiguration/geometry").text = "DIV_B"
    module = root.find("module[@id='edu.tigers.sumatra.vision.AVisionFilter']")
    ET.SubElement(ET.SubElement(module, "properties"), "useThreads").text = "false"
    root.find(".//port").text = str(a.ci_port)
    root.find(".//gameControllerPort").text = str(a.gc_port)
    xml.write(file, encoding="utf-8")
    os.chdir(folder)
    os.execv(
        java,
        [
            java,
            "-Xmx320m",
            "-XX:ActiveProcessorCount=2",
            "-Djava.awt.headless=true",
            "-cp",
            cp,
            "EventObserver",
            a.mode,
        ],
    )


if __name__ == "__main__":
    main()
