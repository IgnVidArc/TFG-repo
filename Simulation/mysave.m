function [] = mysave(name, path)
    defaultname = 'newfigure.pdf';
    defaultpath = ['C:\Users\ignac\OneDrive - Universidad Politécnica de Madrid\' ...
        '04 - GIA 4 - Cuatri 8\TFG\Documento\TesisRestart\Images\'];

    savename = defaultname;
    savepath = defaultpath;
    if nargin > 0
        savename = name;
    end
    if nargin > 1
        savepath = path;
    end
    
    exportgraphics(gcf, [savepath savename])

end