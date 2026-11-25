# 固定赛道统一启动实验

六条件采用同一冻结协议。原目标运动在初始化、有限优化、补点和独立复查中保持同一函数。所有输出是名义规划结果。

|赛道|自由点全程s|中心全程s|自由点节时|自由点成功初值|中心成功初值|自由点计算s|中心计算s|
|---|---:|---:|---:|---:|---:|---:|---:|
splits_moderate|**6.127**|6.391|4.12%|8/8|8/8|53.214|18.311|
splits_high|**6.140**|6.381|3.77%|8/8|8/8|93.196|12.071|
alphapilot_moderate|**10.626**|10.956|3.01%|8/8|8/8|66.925|37.792|
alphapilot_high|**10.542**|10.958|3.79%|8/8|8/8|80.440|46.196|
airsim_moderate|**14.206**|15.058|5.66%|8/8|8/8|536.003|321.023|
airsim_high|14.293|未找到|—|1/8|0/8|1069.899|588.745|

全程时间覆盖原起点、按序穿完所有窗口及原指定终点。三个布局是Split-S 7窗、AlphaPilot 11窗、AirSim 21窗。

计算时间含八路当前目标MINCO时间初始化的公共墙钟、两方法全部PVAJ准备和各自八候选与独立复查的实际并行等待。全部候选结束后按原总目标在合格解中选择。候选CPU时间未累加。

一次完整候选须有限问题正常结束且当前有限约束合格，原SIP复查通过，独立实体窗框碰撞/顺序/终点检查通过。未找到记录不记为无碰撞通过。优化器状态和失败原因在逐候选明细。

原生PVAJ MINCO、TOGT正时间映射、原动力学软惩罚1/1000、33节点/段积分、规划安全球0.30m、有限球与内缩0.3001m、原0.1m/s法向速度条件沿用本批协议。几何仍是浮点连续域数值证据，动力学仍是软约束。

本批初始化采用八个原逐段时间模式，各自在当前目标运动下先做MINCO中心时间优化，再恢复PVAJ。自由/中心对应种子物理轨迹相同。每次正式求解只读原场景，不载入静态、低幅度或其他赛道的已求结果。八候选均保留。

固定的窗口运动参数见上级motion_audit/MOTION_TABLE.md和MOTION_AUDIT.json。每个移动窗的完整参数组合各不相同，静止门按原场景列出。

## 逐候选结果

|条件|候选|分类|全程s|有限求解状态及迭代次数|
|---|---|---|---:|---|
splits_moderate|ours_4|nominal_safe_complete|6.127|Solve_Succeeded(218) Solve_Succeeded(189) Solve_Succeeded(157) Solve_Succeeded(178) Solve_Succeeded(164) Solve_Succeeded(94) Solve_Succeeded(127) Solve_Succeeded(36) |
splits_moderate|ours_1|nominal_safe_complete|6.127|Solve_Succeeded(221) Solve_Succeeded(165) Solve_Succeeded(172) Solve_Succeeded(154) Solve_Succeeded(123) Solve_Succeeded(61) Solve_Succeeded(253) Solve_Succeeded(32) |
splits_moderate|ours_5|nominal_safe_complete|6.127|Solve_Succeeded(161) Solve_Succeeded(124) Solve_Succeeded(199) Solve_Succeeded(202) Solve_Succeeded(253) Solve_Succeeded(70) Solve_Succeeded(187) Solve_Succeeded(42) |
splits_moderate|ours_3|nominal_safe_complete|6.127|Solve_Succeeded(396) Solve_Succeeded(152) Solve_Succeeded(176) Solve_Succeeded(292) Solve_Succeeded(141) Solve_Succeeded(72) Solve_Succeeded(284) Solve_Succeeded(37) |
splits_moderate|ours_6|nominal_safe_complete|6.127|Solve_Succeeded(186) Solve_Succeeded(202) Solve_Succeeded(142) Solve_Succeeded(170) Solve_Succeeded(269) Solve_Succeeded(93) Solve_Succeeded(238) Solve_Succeeded(36) |
splits_moderate|ours_2|nominal_safe_complete|6.127|Solve_Succeeded(336) Solve_Succeeded(157) Solve_Succeeded(171) Solve_Succeeded(324) Solve_Succeeded(160) Solve_Succeeded(239) Solve_Succeeded(215) Solve_Succeeded(36) |
splits_moderate|ours_0|nominal_safe_complete|6.127|Solve_Succeeded(469) Solve_Succeeded(185) Solve_Succeeded(153) Solve_Succeeded(266) Solve_Succeeded(278) Solve_Succeeded(71) Solve_Succeeded(314) Solve_Succeeded(31) |
splits_moderate|ours_7|nominal_safe_complete|6.127|Solve_Succeeded(569) Solve_Succeeded(169) Solve_Succeeded(179) Solve_Succeeded(225) Solve_Succeeded(265) Solve_Succeeded(235) Solve_Succeeded(196) Solve_Succeeded(35) |
splits_moderate|center_5|nominal_safe_complete|6.391|Solve_Succeeded(257) |
splits_moderate|center_4|nominal_safe_complete|6.391|Solve_Succeeded(261) |
splits_moderate|center_0|nominal_safe_complete|6.391|Solve_Succeeded(269) |
splits_moderate|center_6|nominal_safe_complete|6.391|Solve_Succeeded(272) |
splits_moderate|center_7|nominal_safe_complete|6.391|Solve_Succeeded(288) |
splits_moderate|center_3|nominal_safe_complete|6.391|Solve_Succeeded(311) |
splits_moderate|center_1|nominal_safe_complete|6.391|Solve_Succeeded(524) |
splits_moderate|center_2|nominal_safe_complete|6.391|Solve_Succeeded(426) |
splits_high|ours_4|nominal_safe_complete|6.140|Solve_Succeeded(485) Solve_Succeeded(200) Solve_Succeeded(247) Solve_Succeeded(459) Solve_Succeeded(186) Solve_Succeeded(126) Solve_Succeeded(259) Solve_Succeeded(205) Solve_Succeeded(104) |
splits_high|ours_0|nominal_safe_complete|6.140|Solve_Succeeded(458) Solve_Succeeded(246) Solve_Succeeded(262) Solve_Succeeded(511) Solve_Succeeded(138) Solve_Succeeded(181) Solve_Succeeded(172) Solve_Succeeded(131) Solve_Succeeded(132) |
splits_high|ours_1|nominal_safe_complete|6.140|Solve_Succeeded(485) Solve_Succeeded(285) Solve_Succeeded(282) Solve_Succeeded(741) Solve_Succeeded(158) Solve_Succeeded(208) Solve_Succeeded(164) Solve_Succeeded(194) Solve_Succeeded(156) |
splits_high|ours_6|nominal_safe_complete|6.140|Solve_Succeeded(592) Solve_Succeeded(183) Solve_Succeeded(311) Solve_Succeeded(563) Solve_Succeeded(134) Solve_Succeeded(324) Solve_Succeeded(201) Solve_Succeeded(244) Solve_Succeeded(178) |
splits_high|ours_5|nominal_safe_complete|6.140|Solve_Succeeded(715) Solve_Succeeded(290) Solve_Succeeded(451) Solve_Succeeded(572) Solve_Succeeded(130) Solve_Succeeded(173) Solve_Succeeded(202) Solve_Succeeded(199) Solve_Succeeded(196) |
splits_high|ours_7|nominal_safe_complete|6.140|Solve_Succeeded(770) Solve_Succeeded(220) Solve_Succeeded(265) Solve_Succeeded(513) Solve_Succeeded(165) Solve_Succeeded(317) Solve_Succeeded(268) Solve_Succeeded(258) Solve_Succeeded(181) |
splits_high|ours_2|nominal_safe_complete|6.140|Solve_Succeeded(864) Solve_Succeeded(342) Solve_Succeeded(244) Solve_Succeeded(591) Solve_Succeeded(198) Solve_Succeeded(276) Solve_Succeeded(220) Solve_Succeeded(175) Solve_Succeeded(235) |
splits_high|ours_3|nominal_safe_complete|6.140|Solve_Succeeded(802) Solve_Succeeded(266) Solve_Succeeded(313) Solve_Succeeded(895) Solve_Succeeded(145) Solve_Succeeded(307) Solve_Succeeded(342) Solve_Succeeded(166) Solve_Succeeded(254) |
splits_high|center_7|nominal_safe_complete|6.381|Solve_Succeeded(162) |
splits_high|center_5|nominal_safe_complete|6.381|Solve_Succeeded(165) |
splits_high|center_4|nominal_safe_complete|6.381|Solve_Succeeded(216) |
splits_high|center_6|nominal_safe_complete|6.381|Solve_Succeeded(186) |
splits_high|center_1|nominal_safe_complete|6.381|Solve_Succeeded(186) |
splits_high|center_3|nominal_safe_complete|6.381|Solve_Succeeded(279) |
splits_high|center_2|nominal_safe_complete|6.381|Solve_Succeeded(365) |
splits_high|center_0|nominal_safe_complete|6.381|Solve_Succeeded(221) |
alphapilot_moderate|ours_5|nominal_safe_complete|10.626|Solve_Succeeded(418) Solve_Succeeded(211) Solve_Succeeded(137) Solve_Succeeded(102) Solve_Succeeded(85) Solve_Succeeded(76) |
alphapilot_moderate|ours_7|nominal_safe_complete|10.626|Solve_Succeeded(305) Solve_Succeeded(327) Solve_Succeeded(153) Solve_Succeeded(110) Solve_Succeeded(83) Solve_Succeeded(55) |
alphapilot_moderate|ours_3|nominal_safe_complete|10.626|Solve_Succeeded(351) Solve_Succeeded(286) Solve_Succeeded(197) Solve_Succeeded(93) Solve_Succeeded(85) Solve_Succeeded(61) |
alphapilot_moderate|ours_6|nominal_safe_complete|10.626|Solve_Succeeded(506) Solve_Succeeded(236) Solve_Succeeded(117) Solve_Succeeded(117) Solve_Succeeded(75) Solve_Succeeded(53) |
alphapilot_moderate|ours_2|nominal_safe_complete|10.626|Solve_Succeeded(382) Solve_Succeeded(267) Solve_Succeeded(160) Solve_Succeeded(120) Solve_Succeeded(82) Solve_Succeeded(46) |
alphapilot_moderate|ours_0|nominal_safe_complete|10.626|Solve_Succeeded(561) Solve_Succeeded(250) Solve_Succeeded(146) Solve_Succeeded(116) Solve_Succeeded(69) Solve_Succeeded(68) |
alphapilot_moderate|ours_1|nominal_safe_complete|10.626|Solve_Succeeded(581) Solve_Succeeded(239) Solve_Succeeded(203) Solve_Succeeded(108) Solve_Succeeded(74) Solve_Succeeded(104) |
alphapilot_moderate|ours_4|nominal_safe_complete|10.626|Solve_Succeeded(606) Solve_Succeeded(196) Solve_Succeeded(195) Solve_Succeeded(123) Solve_Succeeded(69) Solve_Succeeded(73) |
alphapilot_moderate|center_6|nominal_safe_complete|10.956|Solve_Succeeded(249) |
alphapilot_moderate|center_7|nominal_safe_complete|10.956|Solve_Succeeded(397) |
alphapilot_moderate|center_2|nominal_safe_complete|10.956|Solve_Succeeded(402) |
alphapilot_moderate|center_1|nominal_safe_complete|10.956|Solve_Succeeded(248) |
alphapilot_moderate|center_0|nominal_safe_complete|10.956|Solve_Succeeded(263) |
alphapilot_moderate|center_4|nominal_safe_complete|10.956|Solve_Succeeded(414) |
alphapilot_moderate|center_5|nominal_safe_complete|10.956|Solve_Succeeded(528) |
alphapilot_moderate|center_3|nominal_safe_complete|10.956|Solve_Succeeded(542) |
alphapilot_high|ours_5|nominal_safe_complete|10.542|Solve_Succeeded(270) Solve_Succeeded(175) Solve_Succeeded(114) Solve_Succeeded(97) Solve_Succeeded(70) Solve_Succeeded(106) Solve_Succeeded(80) |
alphapilot_high|ours_0|nominal_safe_complete|10.542|Solve_Succeeded(384) Solve_Succeeded(216) Solve_Succeeded(95) Solve_Succeeded(101) Solve_Succeeded(66) Solve_Succeeded(90) Solve_Succeeded(93) |
alphapilot_high|ours_4|nominal_safe_complete|10.542|Solve_Succeeded(260) Solve_Succeeded(154) Solve_Succeeded(117) Solve_Succeeded(136) Solve_Succeeded(84) Solve_Succeeded(65) Solve_Succeeded(54) |
alphapilot_high|ours_7|nominal_safe_complete|10.543|Solve_Succeeded(348) Solve_Succeeded(239) Solve_Succeeded(279) Solve_Succeeded(122) Solve_Succeeded(74) Solve_Succeeded(72) Solve_Succeeded(67) |
alphapilot_high|ours_6|nominal_safe_complete|10.543|Solve_Succeeded(504) Solve_Succeeded(230) Solve_Succeeded(164) Solve_Succeeded(92) Solve_Succeeded(89) Solve_Succeeded(63) Solve_Succeeded(84) |
alphapilot_high|ours_3|nominal_safe_complete|10.542|Solve_Succeeded(351) Solve_Succeeded(140) Solve_Succeeded(146) Solve_Succeeded(120) Solve_Succeeded(64) Solve_Succeeded(75) Solve_Succeeded(93) |
alphapilot_high|ours_1|nominal_safe_complete|10.543|Solve_Succeeded(615) Solve_Succeeded(175) Solve_Succeeded(220) Solve_Succeeded(94) Solve_Succeeded(56) Solve_Succeeded(93) Solve_Succeeded(91) |
alphapilot_high|ours_2|nominal_safe_complete|10.543|Solve_Succeeded(686) Solve_Succeeded(300) Solve_Succeeded(275) Solve_Succeeded(138) Solve_Succeeded(66) Solve_Succeeded(64) Solve_Succeeded(78) |
alphapilot_high|center_7|nominal_safe_complete|10.958|Solve_Succeeded(445) |
alphapilot_high|center_5|nominal_safe_complete|10.958|Solve_Succeeded(472) |
alphapilot_high|center_6|nominal_safe_complete|10.958|Solve_Succeeded(445) |
alphapilot_high|center_4|nominal_safe_complete|10.958|Solve_Succeeded(756) |
alphapilot_high|center_1|nominal_safe_complete|10.958|Solve_Succeeded(503) |
alphapilot_high|center_3|nominal_safe_complete|10.958|Solve_Succeeded(547) |
alphapilot_high|center_0|nominal_safe_complete|10.958|Solve_Succeeded(728) |
alphapilot_high|center_2|nominal_safe_complete|10.958|Solve_Succeeded(804) |
airsim_moderate|ours_6|nominal_safe_complete|14.206|Solve_Succeeded(670) Solve_Succeeded(368) Solve_Succeeded(476) Solve_Succeeded(284) Solve_Succeeded(175) Solve_Succeeded(177) |
airsim_moderate|ours_7|nominal_safe_complete|14.206|Solve_Succeeded(889) Solve_Succeeded(291) Solve_Succeeded(656) Solve_Succeeded(201) Solve_Succeeded(301) Solve_Succeeded(195) |
airsim_moderate|ours_2|nominal_safe_complete|14.206|Solve_Succeeded(1180) Solve_Succeeded(302) Solve_Succeeded(471) Solve_Succeeded(315) Solve_Succeeded(213) Solve_Succeeded(134) |
airsim_moderate|ours_5|nominal_safe_complete|14.206|Solve_Succeeded(1200) Solve_Succeeded(528) Solve_Succeeded(393) Solve_Succeeded(189) Solve_Succeeded(139) Solve_Succeeded(211) |
airsim_moderate|ours_0|nominal_safe_complete|14.206|Solve_Succeeded(696) Solve_Succeeded(258) Solve_Succeeded(333) Solve_Succeeded(515) Solve_Succeeded(261) Solve_Succeeded(349) |
airsim_moderate|ours_3|nominal_safe_complete|14.206|Solve_Succeeded(605) Solve_Succeeded(333) Solve_Succeeded(637) Solve_Succeeded(345) Solve_Succeeded(148) Solve_Succeeded(145) |
airsim_moderate|ours_4|nominal_safe_complete|14.206|Solve_Succeeded(3911) Solve_Succeeded(298) Solve_Succeeded(342) Solve_Succeeded(240) Solve_Succeeded(144) Solve_Succeeded(177) |
airsim_moderate|ours_1|nominal_safe_complete|14.206|Solve_Succeeded(4064) Solve_Succeeded(316) Solve_Succeeded(672) Solve_Succeeded(347) Solve_Succeeded(178) Solve_Succeeded(107) |
airsim_moderate|center_6|nominal_safe_complete|15.075|Solve_Succeeded(809) |
airsim_moderate|center_7|nominal_safe_complete|15.075|Solve_Succeeded(1312) |
airsim_moderate|center_4|nominal_safe_complete|15.058|Solve_Succeeded(1352) |
airsim_moderate|center_5|nominal_safe_complete|15.064|Solve_Succeeded(1487) |
airsim_moderate|center_1|nominal_safe_complete|15.064|Solve_Succeeded(1573) |
airsim_moderate|center_3|nominal_safe_complete|15.064|Solve_Succeeded(1512) |
airsim_moderate|center_0|nominal_safe_complete|15.058|Solve_Succeeded(1389) |
airsim_moderate|center_2|nominal_safe_complete|15.064|Solve_Succeeded(2332) |
airsim_high|ours_3|not_found|—|Maximum_Iterations_Exceeded(5000) |
airsim_high|ours_7|not_found|—|Maximum_Iterations_Exceeded(5000) |
airsim_high|ours_2|not_found|—|Maximum_Iterations_Exceeded(5000) |
airsim_high|ours_1|not_found|—|Maximum_Iterations_Exceeded(5000) |
airsim_high|ours_0|not_found|—|Maximum_Iterations_Exceeded(5000) |
airsim_high|ours_5|nominal_safe_complete|14.293|Solve_Succeeded(3447) Solve_Succeeded(547) Solve_Succeeded(517) Solved_To_Acceptable_Level(473) Solve_Succeeded(3022) Solve_Succeeded(190) Solve_Succeeded(111) Solve_Succeeded(289) |
airsim_high|ours_4|not_found|—|Maximum_Iterations_Exceeded(5000) |
airsim_high|center_1|not_found|—|Maximum_Iterations_Exceeded(5000) |
airsim_high|center_3|not_found|—|Maximum_Iterations_Exceeded(5000) |
airsim_high|center_2|not_found|—|Maximum_Iterations_Exceeded(5000) |
airsim_high|center_7|not_found|—|Maximum_Iterations_Exceeded(5000) |
airsim_high|center_4|not_found|—|Maximum_Iterations_Exceeded(5000) |
airsim_high|center_0|not_found|—|Maximum_Iterations_Exceeded(5000) |
airsim_high|center_5|not_found|—|Maximum_Iterations_Exceeded(5000) |

## 初始化失败

|条件|时间种子|作者初始化状态|说明|
|---|---:|---|---|
airsim_high|6|-1|双方对应种子在初始化阶段停止，纳入8个尝试的分母|

批次状态为complete，已完成6个条件。

## 本批核对与局限

2026-11-16整批完成。48个当前目标时间初始化中47个产生种子，1个在AirSim高档返回作者优化状态-1，双方对应尝试均记为初始化失败。94个实际SIP候选中81个通过全部复查。

AirSim高档自由点只有seed5通过，其余6个进入SIP的自由点候选和7个中心候选都首轮达到5000次上限，未进入几何补点。中心分类为未找到。当前统一方案在最难条件的初值稳定性仍待解决。

高档选中自由点第3轮是原流程允许的Solved_To_Acceptable_Level，有限约束残差0，末次对偶残差约6.26e-8；下一轮乘子清零。最终第7轮Solve_Succeeded后完整几何与实体/门序/终点复查通过。原判据准确记录在DATA_AUDIT.json与原始progress.jsonl，未将该中间轮标成严格Solve_Succeeded。

81条通过候选均通过原C++ SIP结果不变量检查，记录在SIP_RESULT_CHECKS.log。DATA_AUDIT.json逐项核对相同配对时间/PVAJ、中心初始选点、固定目标、统一设置及选优来源。FREEZE.json中的57项和原历史60项SHA全部一致，FINAL_AUDIT.json记录整批结束后无活动进程。
