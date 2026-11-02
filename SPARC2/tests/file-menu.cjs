const {chromium}=require('@playwright/test');
(async()=>{const b=await chromium.launch();try{
const p=await b.newPage();await p.goto('http://localhost:8081');await p.locator('[data-model-ready="true"]').waitFor();
await p.getByRole('button',{name:'File',exact:true}).click();
const chooser=p.waitForEvent('filechooser');await p.getByRole('menuitem',{name:'Load PCD',exact:true}).click();
await(await chooser).setFiles({name:'sample.pcd',mimeType:'application/octet-stream',buffer:Buffer.from('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA ascii\n0 0 0\n1 0 0\n0 1 0\n')});
await p.getByRole('status').filter({hasText:'3 points'}).waitFor();
await p.getByRole('button',{name:'File',exact:true}).click();await p.getByRole('menuitem',{name:'Settings',exact:true}).click();
await p.getByRole('spinbutton',{name:'Settings processing speed'}).fill('1.3');
await p.getByRole('button',{name:'Apply',exact:true}).click();
await p.waitForFunction(async()=>(await(await fetch('/api/state')).json()).speed===1.3);
if(!await p.getByRole('dialog').isVisible())throw Error('Apply closed dialog');
await p.getByRole('button',{name:'Save & Close'}).click();
await p.getByRole('dialog').waitFor({state:'hidden'});
await p.getByRole('button',{name:'Process',exact:true}).click();
for(const name of ['Pose Finder','Path Planner','Optimizer'])await p.getByText(name,{exact:true}).waitFor();
console.log('PASS: file chooser, PCD parse/render, Apply, Save & Close, Process menu');
}finally{await b.close()}})().catch(e=>{console.error(e);process.exit(1)});
