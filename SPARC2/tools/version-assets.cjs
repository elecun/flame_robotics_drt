const fs=require('node:fs'),crypto=require('node:crypto'),path=require('node:path');
const root=path.resolve(__dirname,'..');
const index=path.join(root,'resource/index.html');
let html=fs.readFileSync(index,'utf8');
for(const ext of ['js','css']){
 const hash=crypto.createHash('sha256').update(fs.readFileSync(path.join(root,'resource/dist/app.'+ext))).digest('hex').slice(0,12);
 html=html.replace(new RegExp('/resource/dist/app\\.'+ext+'(?:\\?v=[a-z0-9]+)?','g'),'/resource/dist/app.'+ext+'?v='+hash);
}
fs.writeFileSync(index,html);
