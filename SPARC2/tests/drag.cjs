const {chromium}=require('@playwright/test');
(async()=>{
 const browser=await chromium.launch();
 try{
 const page=await browser.newPage({viewport:{width:1920,height:1080}});
 await page.goto('http://localhost:8080');
 await page.locator('[data-model-ready="true"]').waitFor();
 // Orbit first, then select a visible model surface using Ctrl + left click.
 await page.mouse.move(1100,500);await page.mouse.down();await page.mouse.move(1140,520,{steps:5});await page.mouse.up();
 let found=false,point;
 for(let y=400;y<850&&!found;y+=50)for(let x=800;x<1500&&!found;x+=50){
  await page.mouse.dblclick(x,y);
  found=await page.locator('.joint-editor').count()>0;if(found)point={x,y};
 }
 if(!found)throw Error('No adjustable link could be selected');
 const name=await page.locator('.joint-editor strong').innerText();
 const specs=await(await page.request.get('http://localhost:8080/api/joints')).json();
 const spec=specs[name];if(!spec)throw Error('Selection did not resolve to editable joint');
 const before=(await(await page.request.get('http://localhost:8080/api/state')).json()).joints[name];
 await page.mouse.move(point.x,point.y);await page.mouse.down();
 await page.mouse.move(point.x+45,point.y-45,{steps:10});
 await page.waitForFunction(async({name,before})=>(await(await fetch('/api/state')).json()).joints[name]!==before,{name,before});
 const increased=(await(await page.request.get('http://localhost:8080/api/state')).json()).joints[name];
 await page.mouse.move(point.x-45,point.y+45,{steps:10});
 await page.waitForFunction(async({name,increased})=>(await(await fetch('/api/state')).json()).joints[name]!==increased,{name,increased});
 await page.mouse.up();
 const final=(await(await page.request.get('http://localhost:8080/api/state')).json()).joints[name];
 if(spec.lower!==null && (final<spec.lower||final>spec.upper))throw Error('Joint limit violated');
 const second=await browser.newPage();await second.goto('http://localhost:8080');
 await second.locator('[data-model-ready="true"]').waitFor();
 const shared=await second.evaluate(async name=>(await(await fetch('/api/state')).json()).joints[name],name);
 if(shared!==final)throw Error('Late join state mismatch');
 await page.mouse.move(600,300);await page.mouse.down();await page.mouse.move(650,330);await page.mouse.up();
 await page.screenshot({path:'work/joint-editor.png'});
 console.log('PASS: orbit, selected left-drag 3D-constrained movement, limits, release and late-join state');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
