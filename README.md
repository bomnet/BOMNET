![logo](https://bomnet.github.io/website_assets/logo.png)

# [BOMNET](https://bomnet.github.io/)
> An OMNeT++ Open-Source BIM-based Plug-In for Automatic Environment Generation.

BOMNET takes IFC files as input and generates two essential files for environment configuration: `omnet.ini` and `environment.xml`. The former contains the global simulation configuration, while the latter includes all 3D objects from the BIM along with their associated materials. If material information is unavailable in the Digital Twin, BOMNET infers materials based on the BIM structure.

## Download, Installation, and Usage
To install BOMNET, it's recommended to download the stable version from the official website and follow the provided tutorial:
- [Download and install BOMNET](https://bomnet.github.io/#down)
- [How to use BOMNET](https://bomnet.github.io/#use)

Make sure to install the **required dependencies** using pip before running the program. Refer to the website for the list of dependencies.

## Development Guide
Once the project is cloned, import each folder in Eclipse IDE or directly on OMNET++ as 'Existing Projects into Workspace'.
To compile the plugin, ensure you have 'Eclipse Plug-in Development Environment' installed to export the project as 'Deployable features'.

## Content
- `BomnetHandler.java`: This is the main class of the project, serving as the entry point of the plugin. It includes the file processing logic and calls to the Python script.
- `generate_files.py`: Contains the IFC extraction and files generation processes.
- `BOMNET_*` files: These files are meant to be either added or overwritten into INET.

## Credits and Contact

This project is an academic project supervised by [Professor Abderrezak Rachedi](https://igm.univ-mlv.fr/~rachedi/), PhD canditates [Aurélien Chambon](https://aurelienchambon.pythonanywhere.com/en/) and [Clément Dutriez](https://www.linkedin.com/in/cl%C3%A9ment-dutriez/) with the help of Master degree students [Abderrahmane Melek](https://www.linkedin.com/in/melek-abderrahmane/), [Khalifa Serraye](https://khalifaserraye.github.io/portfolio/), Mohammed Menacer and [Louis Thidet](https://louis-thidet.github.io/) as a M2-SSIO project in Université Gustave Eiffel, France.

For any questions, please contact us at contact.bomnet@gmail.com.
