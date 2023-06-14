%% EXPORTING Simulink MODELS AS PDF:
MODEL = 'pdf_UAVNLCL_TRACK_ALL';
PATH = 'C:\Users\ignac\OneDrive - Universidad Politécnica de Madrid\04 - GIA 4 - Cuatri 8\TFG\Documento\TesisRestart\Images\';
NAME = 'simu_TRACK_ALL.pdf';
handle = load_system(MODEL);
% saveas(get_param(handle,'Handle'), [PATH, NAME])