const {chromium}=require('@playwright/test');
(async()=>{
 const browser=await chromium.launch();
 try{
 const page=await browser.newPage({viewport:{width:1920,height:1080}});
 await page.goto('http://localhost:8080');
 await page.locator('[data-model-ready="true"]').waitFor();
 // Orbit first, then select a visible model surface using Ctrl + left click.
 await page.mouse.move(1100,500);await page.mouse.down();await page.mouse.move(1140,520,{steps:5});await page.mouse.up();
 let found=false;
 for(let y=400;y<850&&!found;y+=50)for(let x=800;x<1500&&!found;x+=50){
  await page.mouse.dblclick(x,y);
  found=await page.locator('.joint-editor').count()>0;
 }
 if(!found)throw Error('No adjustable link could be selected');
 const name=await page.locator('.joint-editor strong').innerText();
 const specs=await(await page.request.get('http://localhost:8080/api/joints')).json();
 const spec=specs[name];if(!spec)throw Error('Selection did not resolve to editable joint');
 const value=spec.lower===null?.25:(spec.lower+spec.upper)/2;
 const input=page.getByRole('spinbutton',{name:'Joint value'});
 await input.fill(String(value));await input.press('Tab');
 await page.waitForFunction(async({name,value})=>{const s=await(await fetch('/api/state')).json();return s.joints[name]===value;},{name,value});

 await page.screenshot({path:'work/joint-editor.png'});
 const selectedImage=await page.locator('.viewport canvas').screenshot();
 await page.getByRole('button',{name:'Close',exact:true}).click();
 if(await page.locator('.joint-editor').count())throw Error('Selection was not cleared');
 const clearedImage=await page.locator('.viewport canvas').screenshot();
 if(selectedImage.equals(clearedImage))throw Error('Highlight did not change after clearing');
 await page.mouse.dblclick(700,150);
 if(await page.locator('.joint-editor').count())throw Error('Empty double click selected a model');

 console.log('PASS: orbit, double-click raycast selection, joint input and shared server update');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
