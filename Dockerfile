FROM mcr.microsoft.com/windows/servercore:ltsc2019

RUN powershell -Command \
    Invoke-WebRequest -Uri http://download.osgeo.org/osgeo4w/osgeo4w-setup-x86_64.exe -OutFile osgeo4w-setup-x86_64.exe; \
    Start-Process -Wait -FilePath ./osgeo4w-setup-x86_64.exe -ArgumentList '/S'

ENV PATH="C:/OSGeo4W64/bin:${PATH}"

WORKDIR /

COPY . .

CMD ["C:/OSGeo4W64/bin/osgeo4w.bat", "python", "mrsid_transfer.py"]
