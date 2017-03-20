%%% Hancock Hall Experiment 03/09
%%% Data Processing and Visualization
% Clear Workspace
clc; clear all; close all;
% Load Measurements
load('hancock_03_09.mat')
load('1019.mat')
load('3032.mat')
% Merge measurements to create measurement space
measurement_space = [data_1019(:,2:25); data_3032(:,2:25)];
% Create grid labels for each measurement vector
grid_labels_1019 = zeros(1313,2);
grid_labels_3032 = zeros(230,2);

grid_labels_1019(1:9,:) = repmat([1,6], [9,1]);
grid_labels_1019(10:18,:) = repmat([2,6], [9,1]);
grid_labels_1019(19:30,:) = repmat([3,6], [12,1]);
grid_labels_1019(31:39,:) = repmat([4,6], [9,1]);
grid_labels_1019(40:51,:) = repmat([5,6],[12,1]);
grid_labels_1019(52:60,:) = repmat([6,6],[9,1]);
grid_labels_1019(61:69,:) = repmat([7,6], [9,1]);
grid_labels_1019(70:78,:) = repmat([8,6],[9,1]);
grid_labels_1019(79:87,:) = repmat([9,6],[9,1]);
grid_labels_1019(88:96,:) = repmat([10,6],[9,1]);
grid_labels_1019(97:105,:) = repmat([11,6],[9,1]);
grid_labels_1019(106:114,:) = repmat([12,6],[9,1]);
grid_labels_1019(115:123,:) = repmat([13,6],[9,1]);
grid_labels_1019(124:132,:) = repmat([14,6],[9,1]);
grid_labels_1019(133:141,:) = repmat([15,6],[9,1]);
grid_labels_1019(142:150,:) = repmat([16,6],[9,1]);
grid_labels_1019(151:159,:) = repmat([17,6],[9,1]);
grid_labels_1019(160:168,:) = repmat([18,6],[9,1]);
grid_labels_1019(169:177,:) = repmat([19,6],[9,1]);
grid_labels_1019(178:186,:) = repmat([20,6],[9,1]);
grid_labels_1019(187:195,:) = repmat([21,6],[9,1]);
grid_labels_1019(196:204,:) = repmat([22,6],[9,1]);
grid_labels_1019(205:213,:) = repmat([23,6],[9,1]);
grid_labels_1019(214:222,:) = repmat([24,6],[9,1]);
grid_labels_1019(223:231,:) = repmat([25,6],[9,1]);
grid_labels_1019(232:240,:) = repmat([22,5],[9,1]);
grid_labels_1019(241:249,:) = repmat([21,5],[9,1]);
grid_labels_1019(250:258,:) = repmat([20,5],[9,1]);
grid_labels_1019(259:267,:) = repmat([19,5],[9,1]);
grid_labels_1019(268:276,:) = repmat([18,5],[9,1]);
grid_labels_1019(277:285,:) = repmat([17,5],[9,1]);
grid_labels_1019(286:294,:) = repmat([16,5],[9,1]);
grid_labels_1019(295:303,:) = repmat([15,5],[9,1]);
grid_labels_1019(304:312,:) = repmat([14,5],[9,1]);
grid_labels_1019(313:321,:) = repmat([13,5],[9,1]);
grid_labels_1019(322:330,:) = repmat([12,5],[9,1]);
grid_labels_1019(331:339,:) = repmat([11,5],[9,1]);
grid_labels_1019(340:348,:) = repmat([10,5],[9,1]);
grid_labels_1019(349:357,:) = repmat([9,5],[9,1]);
grid_labels_1019(358:366,:) = repmat([8,5],[9,1]);
grid_labels_1019(367:375,:) = repmat([7,5],[9,1]);
grid_labels_1019(376:386,:) = repmat([6,5],[11,1]);
grid_labels_1019(387:395,:) = repmat([5,5],[9,1]);
grid_labels_1019(396:404,:) = repmat([4,5],[9,1]);
grid_labels_1019(405:413,:) = repmat([3,5],[9,1]);
grid_labels_1019(414:422,:) = repmat([2,5],[9,1]);
grid_labels_1019(423:431,:) = repmat([1,5],[9,1]);
grid_labels_1019(432:440,:) = repmat([1,4],[9,1]);
grid_labels_1019(441:451,:) = repmat([2,4],[11,1]);
grid_labels_1019(452:460,:) = repmat([3,4],[9,1]);
grid_labels_1019(461:469,:) = repmat([4,4],[9,1]);
grid_labels_1019(470:478,:) = repmat([5,4],[9,1]);
grid_labels_1019(479:487,:) = repmat([6,4],[9,1]);
grid_labels_1019(488:496,:) = repmat([7,4],[9,1]);
grid_labels_1019(497:505,:) = repmat([8,4],[9,1]);
grid_labels_1019(506:514,:) = repmat([9,4],[9,1]);
grid_labels_1019(515:523,:) = repmat([10,4],[9,1]);
grid_labels_1019(524:532,:) = repmat([11,4],[9,1]);
grid_labels_1019(533:542,:) = repmat([12,4],[10,1]);
grid_labels_1019(543:551,:) = repmat([13,4],[9,1]);
grid_labels_1019(552:560,:) = repmat([14,4],[9,1]);
grid_labels_1019(561:569,:) = repmat([15,4],[9,1]);
grid_labels_1019(570:578,:) = repmat([16,4],[9,1]);
grid_labels_1019(579:590,:) = repmat([17,4],[12,1]);
grid_labels_1019(591:607,:) = repmat([18,4],[17,1]);
grid_labels_1019(608:616,:) = repmat([19,4],[9,1]);
grid_labels_1019(617:628,:) = repmat([20,4],[12,1]);
grid_labels_1019(629:637,:) = repmat([21,4],[9,1]);
grid_labels_1019(638:646,:) = repmat([22,4],[9,1]);
grid_labels_1019(647:655,:) = repmat([25,2],[9,1]);
grid_labels_1019(656:664,:) = repmat([24,2],[9,1]);
grid_labels_1019(665:673,:) = repmat([23,2],[9,1]);
grid_labels_1019(674:682,:) = repmat([22,2],[9,1]);
grid_labels_1019(683:691,:) = repmat([22,3],[9,1]);
grid_labels_1019(692:700,:) = repmat([21,3],[9,1]);
grid_labels_1019(701:709,:) = repmat([21,2],[9,1]);
grid_labels_1019(710:721,:) = repmat([20,2],[12,1]);
grid_labels_1019(722:730,:) = repmat([20,3],[9,1]);
grid_labels_1019(731:739,:) = repmat([19,3],[9,1]);
grid_labels_1019(740:748,:) = repmat([19,2],[9,1]);
grid_labels_1019(749:757,:) = repmat([18,2],[9,1]);
grid_labels_1019(758:766,:) = repmat([18,3],[9,1]);
grid_labels_1019(767:778,:) = repmat([17,2],[12,1]);
grid_labels_1019(779:787,:) = repmat([16,2],[9,1]);
grid_labels_1019(788:799,:) = repmat([16,3],[12,1]);
grid_labels_1019(800:808,:) = repmat([15,3],[9,1]);
grid_labels_1019(809:817,:) = repmat([15,2],[9,1]);
grid_labels_1019(818:829,:) = repmat([14,2],[12,1]);
grid_labels_1019(830:838,:) = repmat([14,3],[9,1]);
grid_labels_1019(839:849,:) = repmat([13,3],[11,1]);
grid_labels_1019(850:858,:) = repmat([13,2],[9,1]);
grid_labels_1019(859:867,:) = repmat([12,2],[9,1]);
grid_labels_1019(868:879,:) = repmat([12,3],[12,1]);
grid_labels_1019(880:888,:) = repmat([11,3],[9,1]);
grid_labels_1019(889:897,:) = repmat([11,2],[9,1]);
grid_labels_1019(898:909,:) = repmat([10,2],[12,1]);
grid_labels_1019(910:918,:) = repmat([10,3],[9,1]);
grid_labels_1019(919:927,:) = repmat([9,3],[9,1]);
grid_labels_1019(928:938,:) = repmat([9,2],[11,1]);
grid_labels_1019(939:947,:) = repmat([8,2],[9,1]);
grid_labels_1019(948:956,:) = repmat([8,3],[9,1]);
grid_labels_1019(957:965,:) = repmat([7,2],[9,1]);
grid_labels_1019(966:974,:) = repmat([6,2],[9,1]);
grid_labels_1019(975:983,:) = repmat([6,3],[9,1]);
grid_labels_1019(984:992,:) = repmat([5,3],[9,1]);
grid_labels_1019(993:1000,:) = repmat([5,2],[8,1]);
grid_labels_1019(1001:1009,:) = repmat([4,2],[9,1]);
grid_labels_1019(1010:1018,:) = repmat([4,3],[9,1]);
grid_labels_1019(1019:1027,:) = repmat([3,3],[9,1]);
grid_labels_1019(1028:1036,:) = repmat([3,2],[9,1]);
grid_labels_1019(1037:1045,:) = repmat([2,2],[9,1]);
grid_labels_1019(1046:1054,:) = repmat([2,3],[9,1]);
grid_labels_1019(1055:1063,:) = repmat([1,3],[9,1]);
grid_labels_1019(1064:1072,:) = repmat([1,2],[9,1]);
grid_labels_1019(1073:1081,:) = repmat([1,1],[9,1]);
grid_labels_1019(1082:1090,:) = repmat([2,1],[9,1]);
grid_labels_1019(1091:1099,:) = repmat([3,1],[9,1]);
grid_labels_1019(1100:1108,:) = repmat([4,1],[9,1]);
grid_labels_1019(1109:1120,:) = repmat([5,1],[12,1]);
grid_labels_1019(1121:1129,:) = repmat([6,1],[9,1]);
grid_labels_1019(1130:1138,:) = repmat([7,1],[9,1]);
grid_labels_1019(1139:1147,:) = repmat([8,1],[9,1]);
grid_labels_1019(1148:1159,:) = repmat([9,1],[12,1]);
grid_labels_1019(1160:1170,:) = repmat([10,1],[11,1]);
grid_labels_1019(1171:1179,:) = repmat([11,1],[9,1]);
grid_labels_1019(1180:1188,:) = repmat([12,1],[9,1]);
grid_labels_1019(1189:1199,:) = repmat([13,1],[11,1]);
grid_labels_1019(1200:1208,:) = repmat([14,1],[9,1]);
grid_labels_1019(1209:1217,:) = repmat([16,1],[9,1]);
grid_labels_1019(1218:1226,:) = repmat([15,1],[9,1]);
grid_labels_1019(1227:1235,:) = repmat([17,1],[9,1]);
grid_labels_1019(1236:1244,:) = repmat([18,1],[9,1]);
grid_labels_1019(1245:1253,:) = repmat([19,1],[9,1]);
grid_labels_1019(1254:1268,:) = repmat([20,1],[15,1]);
grid_labels_1019(1269:1277,:) = repmat([21,1],[9,1]);
grid_labels_1019(1278:1286,:) = repmat([22,1],[9,1]);
grid_labels_1019(1287:1295,:) = repmat([23,1],[9,1]);
grid_labels_1019(1296:1304,:) = repmat([24,1],[9,1]);
grid_labels_1019(1305:1313,:) = repmat([25,1],[9,1]);

grid_labels_3032(220:230,:) = repmat([1,7],[11,1]);
grid_labels_3032(211:219,:) = repmat([2,7],[9,1]);
grid_labels_3032(202:210,:) = repmat([3,7],[9,1]);
grid_labels_3032(193:201,:) = repmat([4,7],[9,1]);
grid_labels_3032(184:192,:) = repmat([5,7],[9,1]);
grid_labels_3032(175:183,:) = repmat([6,7],[9,1]);
grid_labels_3032(166:174,:) = repmat([7,7],[9,1]);
grid_labels_3032(157:165,:) = repmat([8,7],[9,1]);
grid_labels_3032(148:156,:) = repmat([9,7],[9,1]);
grid_labels_3032(136:149,:) = repmat([10,7],[14,1]);
grid_labels_3032(127:135,:) = repmat([11,7],[9,1]);
grid_labels_3032(118:126,:) = repmat([12,7],[9,1]);
grid_labels_3032(109:117,:) = repmat([13,7],[9,1]);
grid_labels_3032(100:108,:) = repmat([14,7],[9,1]);
grid_labels_3032(91:99,:) = repmat([15,7],[9,1]);
grid_labels_3032(82:90,:) = repmat([16,7],[9,1]);
grid_labels_3032(73:81,:) = repmat([17,7],[9,1]);
grid_labels_3032(64:72,:) = repmat([18,7],[9,1]);
grid_labels_3032(55:63,:) = repmat([19,7],[9,1]);
grid_labels_3032(46:54,:) = repmat([20,7],[9,1]);
grid_labels_3032(37:45,:) = repmat([21,7],[9,1]);
grid_labels_3032(28:36,:) = repmat([22,7],[9,1]);
grid_labels_3032(19:27,:) = repmat([23,7],[9,1]);
grid_labels_3032(10:18,:) = repmat([24,7],[9,1]);
grid_labels_3032(1:9,:) = repmat([25,7],[9,1]);
% merge grid labels
grid_labels = [grid_labels_1019; grid_labels_3032];
grid_labels = [grid_labels(:,2), grid_labels(:,1)];
% create spatial-propagation maps
propagation_maps = zeros(25, 7, 24);
n_obs = zeros(25, 7, 24);
data_wifi = data_total(:,2:9);
data_bt = data_total(:,10:17);
data_lora = data_total(:,18:25);
% the positions of the anchor nodes
pos_node = [0,0; 0, 8; 0, 16; 0, 25; 8, 25; 8, 16; 8, 8; 8, 0];

std_lora = std(data_lora);
%%
% grid_labels_classifier = zeros(1543,175);
% [rows, cols] = size(grid_labels);
grid_labels_linear = (grid_labels(:,1)-1)+(grid_labels(:,2)-1)*7;
% for i=1:rows
%         grid_labels_classifier(i,g(i)+1) = 1;
% end
%% Create Propagation Maps 

for i=1:1313
    grid_cell = grid_labels_1019(i,:);
    propagation_maps(grid_cell(1), grid_cell(2), :) = propagation_maps(grid_cell(1), grid_cell(2), :) + reshape(data_1019(i,2:25), [1, 1, 24]);
    idx = data_1019(i, 2:25) ~= 0;
    n_obs(grid_cell(1), grid_cell(2), idx) = n_obs(grid_cell(1), grid_cell(2), idx) + 1;
end

for i=1:230
    grid_cell = grid_labels_3032(i,:);
    propagation_maps(grid_cell(1), grid_cell(2), :) = propagation_maps(grid_cell(1), grid_cell(2), :) + reshape(data_3032(i,2:25), [1, 1, 24]);
    idx = data_3032(i, 2:25) ~= 0;
    n_obs(grid_cell(1), grid_cell(2), idx) = n_obs(grid_cell(1), grid_cell(2), idx) + 1;
end
clear grid;
propagation_maps = propagation_maps ./ n_obs;
propagation_maps(isnan(propagation_maps)) = 0;
%% Save Measurement Space and Grid-cell Labels
grid_labels = grid_labels - 1;
save('hancock_data.mat', 'data_wifi' ,'data_bt', 'data_lora', 'grid_labels_linear', 'grid_labels');
grid_labels = grid_labels + 1;

%% Clear up unnecessary variables
clear i idx data_1019 data_3032 grid_labels_1019 grid_labels_3032 n_obs grid_cell measurement_space