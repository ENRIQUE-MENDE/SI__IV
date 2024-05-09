clc; 
clear; 
close all;
cam = webcam(1);
lambda = 0.5;

X=0;
Y=0;
kp=.3;
ki=0.00005;
kd=0.1412;
c=[0; 0];
kpy=.2;
kiy=0.00004;
kdy=0.004;

h=1080;
w=1920;
cumErrorx =0;
lastErrorx=0;
cumErrory =0;
lastErrory=0;
rateErrorx=0;
rateErrory=0;
a = arduino("COM7",'mega2560');
sx=servo(a,'A8');
sy=servo(a,'A9');
angulox=90;
anguloy=90;
writePosition(sx,0.5);
writePosition(sy,0.5);

tic
while true
    t=toc;
    snapshot1 = im2double(snapshot(cam));
    [M,N,O] = size(snapshot1);
    img1 = snapshot1 + lambda * randn(M,N,O);
    img1= im2uint8(img1);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Análisis de frecuencia
    for k=1:O
        SNAPSHOT2(:,:,k)=fftshift(fft2(img1(:,:,k)));
    end

    %% Creación de filtro en frecuencia
    H=zeros(M,N);
    r=8;
    for kx=1:M
        for ky=1:N
            if (kx-M/2)^2+(ky-N/2)^2<r^2
                H(kx,ky)=1;
            end
        end
    end

    %% Filtrado en frecuencia
    for k=1:O
        SNAPSHOT_FILTRADO(:,:,k)=SNAPSHOT2(:,:,k).*H;
    end

    %% Regresar al dominio espacial
    for k=1:O
        img0(:,:,k)=ifft2(ifftshift(SNAPSHOT_FILTRADO(:,:,k)));
    end
    img0=real(img0);
    img0 = im2uint8(img0); % Convertir la imagen resultante a uint8

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    img = imsubtract(img0(:,:,3), rgb2gray(img0)); % Diferencia del canal azul y la escala de grises
    blue_threshold = 40; % Umbral para determinar la cercanía al color azul
    blue_mask = img > blue_threshold; % Crear una máscara de píxeles azules
    bw = medfilt2(blue_mask); % Filtrar la máscara
    bw = imopen(bw, strel('disk', 1)); % Operación de apertura
    bw = bwareaopen(bw, 3000); % Eliminar áreas pequeñas
    bw = imfill(bw, 'holes'); % Rellenar agujeros
    [L, N] = bwlabel(bw); % Etiquetar regiones

    %-----------------regionprops------------------
    prop = regionprops(L);
    %----------------------------------------------
    % imshow(blue_mask);
    imshow(img0);


    for n = 1:N
        c = round(prop(n).Centroid); % Obtener centroide
        X=c(1);
        Y=c(2);
        rectangle('Position', prop(n).BoundingBox, 'EdgeColor', 'g', 'LineWidth', 2); % Dibujar rectángulo
           end

    cy=Y;
    cx=X;
    % imshow(img0);
 

    hold on;
    plot(cx,cy,"ro")
    plot(w/2,h/2,"ro")
    errorx=((w/2)-cx)/31.7;
    errory=((h/2)-cy)/14.87;%14.87
    if t>.5
        cumErrorx = cumErrorx + errorx*t;
        rateErrorx = (errorx - lastErrorx)/t;
        cumErrory = cumErrory + errory*t;
        rateErrory = (errory - lastErrory)/t;
        lastErrorx=errorx;
        lastErrory=errory;
    end
    outputx = kp*errorx + ki*cumErrorx + kd*rateErrorx;
    outputy = kpy*errory + kiy*cumErrory + kdy*rateErrory;

    controly=(anguloy - outputy)/180;
    controlx=(angulox + outputx)/180;
    if (controlx>1)
        controlx=1;
    end
    if (controlx<0)
        controlx=0;
    end
    if (controly>1)
        controly=1;
    end
    if (controly<0)
        controly=0;
    end
    writePosition(sx,controlx);
    writePosition(sy,controly);
    angulox=controlx*180;
    anguloy=controly*180;
    pause(0.001);
end

