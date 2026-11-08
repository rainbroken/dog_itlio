import {test} from 'node:test';
import assert from 'node:assert/strict';
import {localizationFeedback} from '../web/console/localization_feedback.js';
test('a fresh status with no scan must not imply an active global search',()=>{
 const text=localizationFeedback({fresh:true,input_fresh:false,state:'global_relocalizing',reason:'global_relocalization_requested',localization_source:'historical_keyframes'});
 assert(text.includes('等待新鲜的 LIO'));assert(!text.includes('历史关键帧匹配'));
});
test('searching, retry failure, consecutive confirmations and success are distinguishable',()=>{
 const status={fresh:true,input_fresh:true,state:'global_relocalizing'};
 assert(localizationFeedback({...status,registration_running:true}).includes('正在配准'));
 assert(localizationFeedback({...status,reason:'global_time_budget_exceeded'}).includes('全局搜索超时'));
 assert(localizationFeedback({...status,reason:'global_no_verified_candidate'}).includes('未找到通过验证'));
 assert(localizationFeedback({...status,state:'relocalizing',reason:'accepted',confirmations:2}).includes('连续确认 2 帧'));
 assert(localizationFeedback({...status,valid:true,localization_source:'historical_keyframes'}).includes('已确认定位 · 历史关键帧匹配'));
 assert(localizationFeedback({...status,fresh:false}).includes('未就绪或状态已过期'));
});
