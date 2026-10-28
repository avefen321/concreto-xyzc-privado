// Geometry, C, arithmetic and acceptance remain owned by the server.
export function setupOptimization({getRequest,onSelected}) {
  const $=id=>document.getElementById(id);
  let job=null,requestSource=null,requestConfig=null,polling=false,active=false,info=null,lastData=null,viewBusy=false;
  const escape=value=>String(value??'—').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const parameters=(value,mode)=>!value?'Sin búsqueda':Object.entries(value).filter(([k])=>mode!=='FROZEN_MACHINE_PLAN'||k==='candidate_window').map(([k,v])=>`${({candidate_window:'Alcance',split_level:'Refinamiento',max_sweep_deg:'Arco máx.'})[k]||k}: ${v}${k==='max_sweep_deg'?'°':''}`).join(' · ');
  const show=text=>{$('optimization-status').textContent=text;};
  async function api(path,body) {
    const response=await fetch(path,body===undefined?{cache:'no-store'}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    let data;
    try {data=await response.json();} catch {throw Error('El servidor no ofrece la API de optimización actualizada. Reinicie ABRIR_PROGRAMA.bat.');}
    if(!response.ok)throw Error(typeof data.detail==='string'?data.detail:`HTTP ${response.status}`);
    return data;
  }
  function controls() {
    $('optimization-start').disabled=active||viewBusy||!info?.enabled||!getRequest();
    $('optimization-cancel').disabled=!active;
    $('optimization-method').disabled=active;
    $('optimization-apply').disabled=active||viewBusy||!info?.enabled||!getRequest()||!lastData?.best;
  }
  function renderCandidates(data) {
    const panel=$('optimization-candidates');if(!panel)return;
    const rows=data.candidates||[];panel.hidden=!rows.length;
    const base=data.baseline?.N_blocks;
    const recommended=rows.find(r=>r.recommended);
    $('optimization-recommendation').textContent=recommended?`Mejor candidata válida: ${recommended.label}. ${recommended.id==='constructor'?'La búsqueda no encontró una opción mejor que el constructor fijo. ':recommended.id==='baseline'?'No se encontró una mejora válida de la conversión original. ':''}Puedes verla o descargar su G-code directamente.`:'Validando las referencias…';
    $('optimization-candidates-body').innerHTML=rows.map(row=>{
      const origin=(row.id==='baseline'?'Referencia':row.id==='constructor'?'Geometría fija':row.acquisition?.startsWith('INITIAL')?'Inicial':row.acquisition==='UNIFORM_RANDOM'?'Aleatoria':row.acquisition==='TPE_DENSITY_RATIO'?'IA · TPE':'IA · GP/EI')+(row.duplicate?' · repetida':'');
      const state=row.recommended?'★ Recomendada':!row.valid?'Inválida':row.eligible?'Válida':'Válida · no recomendada';
      const url=`/api/optimization/${job}/candidate/${encodeURIComponent(row.id)}`;
      return `<tr class="${row.recommended?'candidate-best':''}"><td>${escape(row.label)}</td><td>${escape(origin)}</td><td>${escape(parameters(row.parameters,data.reference_mode))}</td><td>${escape(row.N_blocks)}</td><td>${base&&row.N_blocks!=null?(100*(base-row.N_blocks)/base).toFixed(2)+' %':'—'}</td><td>${row.M_microblocks!=null?row.M_microblocks+(row.arc_microblocks||0):'—'}</td><td>${row.xy_error_mm?.toFixed(4)??'—'} mm / ${row.C_error_deg?.toFixed(4)??'—'}°</td><td title="${escape(row.reason||'')}">${escape(state)}</td><td>${row.valid?`<button type="button" data-candidate="${escape(row.id)}" ${active||viewBusy?'disabled':''}>Ver</button> <a href="${url}/gcode" download>↓ G-code</a>`:'Sin salida válida'}</td></tr>`;
    }).join('');
    const best=$('optimization-best-download');best.hidden=active||!rows.some(r=>r.recommended&&r.valid);
    best.href=`/api/optimization/${job}/candidate/best/gcode`;
  }
  function readiness() {
    controls();
    if(!active&&info?.enabled&&!getRequest()) {
      $('optimization-apply').disabled=true;$('optimization-artifacts').hidden=true;
      show('Disponible. Cargue y convierta un G-code para iniciar la búsqueda.');
    }
  }
  const phases={REFERENCE:'Congelando y validando la conversión',CONSTRUCTOR:'Construyendo candidatas',SEARCH:'Evaluando configuraciones',DONE:'Finalizado'};
  function render(data) {
    lastData=data;active=['QUEUED','RUNNING'].includes(data.status);
    const m=data.best,a=data.baseline;
    const names={QUEUED:'En cola',RUNNING:'En ejecución',COMPLETED:'Completado',CANCELLED:'Detenido',FAILED:'Error'};
    show(`${names[data.status]||data.status} · ${phases[data.phase]||''} · ${data.completed}/${data.budget} evaluaciones${data.error?' · '+data.error:''}`);
    if(m&&a) {
      const reduction=a.N_blocks?100*(a.N_blocks-m.N_blocks)/a.N_blocks:0;
      $('ai-optimization-metrics').textContent=`Bloques: ${a.N_blocks} → ${m.N_blocks} (${reduction.toFixed(2)} % de reducción). Microbloques G1 + arcos: ${a.M_microblocks+(a.arc_microblocks||0)} → ${m.M_microblocks+(m.arc_microblocks||0)}. Cota XY: ${m.xy_error_mm?.toFixed(4)??'—'} mm; C: ${m.C_error_deg?.toFixed(4)??'—'}°. Cobertura: ${m.coverage_complete?'completa':'incompleta'}.`;
      $('optimization-apply').disabled=active||viewBusy;
    }
    if(data.summary) {
      const s=data.summary,t=s.timing;
      $('optimization-timing').textContent=t?.status==='ESTIMATED'?`Tiempo cinemático estimado: ${t.t_sim_s.toFixed(2)} s (${t.policy}). Tiempo real: sin medir.`:'Tiempo cinemático: sin configurar límites XYZC. Tiempo real: sin medir.';
      show($('optimization-status').textContent+` · ${s.unique_configurations} configuraciones distintas; ${s.adaptive_evaluations} propuestas adaptativas. ${s.outcome==='NO_VALID_REDUCTION'?'No se encontró una reducción válida; se conserva la conversión. ':''}${s.note||''}`);
    }
    active=['QUEUED','RUNNING'].includes(data.status);controls();
    $('optimization-artifacts').hidden=active||!m;
    $('optimization-json').href=`/api/optimization/${job}/artifact/selected_report.json`;
    $('optimization-csv').href=`/api/optimization/${job}/artifact/history.csv`;
    $('optimization-csv').hidden=!data.completed;
    renderCandidates(data);
    return active;
  }
  async function poll() {
    if(polling)return;
    polling=true;let again=false;
    try {again=render(await api(`/api/optimization/${job}`));}
    catch(error){show(error.message+' Puede volver a consultar la disponibilidad.');active=false;controls();}
    finally{polling=false;if(again)setTimeout(poll,1000);}
  }
  $('optimization-start').onclick=async()=>{
    try {
      const base=getRequest();if(!base)throw Error('Convierta primero un archivo válido.');
      const payload={...base,method:$('optimization-method').value,budget:Number($('optimization-budget').value),seed:Number($('optimization-seed').value)};
      if($('optimization-timing-enabled').checked) {
        payload.limits=Object.fromEntries(['X','Y','Z','C'].map(axis=>{
          const velocity=Number($(`timing-${axis}-v`).value),acceleration=Number($(`timing-${axis}-a`).value);
          if(!(velocity>0&&acceleration>0))throw Error(`Complete velocidad y aceleración positivas del eje ${axis}.`);
          return [axis,{velocity,acceleration}];
        }));
        payload.timing_policy=$('optimization-timing-policy').value;
      }
      active=true;controls();$('optimization-apply').disabled=true;
      lastData=null;if($('optimization-candidates'))$('optimization-candidates').hidden=true;
      if($('optimization-best-download'))$('optimization-best-download').hidden=true;
      $('ai-optimization-metrics').textContent='';$('optimization-artifacts').hidden=true;
      requestSource=base.source;requestConfig=JSON.stringify(base.config);
      show('Iniciando optimización de la conversión actual…');
      const data=await api('/api/optimization',payload);job=data.id;await poll();
    }catch(error){show(error.message);active=false;controls();}
  };
  $('optimization-cancel').onclick=async()=>{try{await api(`/api/optimization/${job}/cancel`,{});show('Cancelación solicitada; se conserva la mejor candidata validada.');}catch(error){show(error.message);}};
  async function openCandidate(candidateId='best') {
    try {
      const current=getRequest();
      if(current?.source!==requestSource||JSON.stringify(current?.config)!==requestConfig)throw Error('El archivo o la configuración cambió: inicie una búsqueda nueva.');
      viewBusy=true;controls();if(lastData)renderCandidates(lastData);
      show('Preparando y verificando la candidata elegida…');
      const result=await api(lastData?.candidates?.length?`/api/optimization/${job}/candidate/${encodeURIComponent(candidateId)}/result`:`/api/optimization/${job}/result`);
      await onSelected(result);
      const chosen=result.report?.optimization_selection;
      if($('optimization-viewing'))$('optimization-viewing').textContent=chosen?`En el visor: ${chosen.label} · ${chosen.N_blocks} bloques${chosen.recommended?' · mejor candidata válida':' · selección de consulta'}. Las descargas del simulador corresponden a esta selección.`:'Candidata cargada en el simulador.';
      show(chosen?`${chosen.label} cargada en el simulador. ${chosen.recommended?'Es la recomendada.':'La recomendación de la búsqueda se conserva.'}`:'Candidata cargada en el simulador.');
    }catch(error){show(error.message);}finally{viewBusy=false;controls();if(lastData)renderCandidates(lastData);}
  }
  $('optimization-apply').onclick=()=>openCandidate('best');
  $('optimization-candidates-body')?.addEventListener('click',event=>{
    const button=event.target.closest('button[data-candidate]');if(button&&!button.disabled)openCandidate(button.dataset.candidate);
  });
  async function refresh() {
    try {
      info=await api('/api/optimization/capabilities');
      for(const option of $('optimization-method').options) {
        option.disabled=!info.algorithms[option.value];option.title=info.dependencies?.[option.value]?.reason||'';
      }
      if(!info.algorithms[$('optimization-method').value])$('optimization-method').value='random';
      const missing=['bo','tpe'].filter(k=>!info.algorithms[k]);
      $('optimization-environment').textContent=missing.length?`Faltan ${missing.join(' / ')} en el Python del servidor (${info.python_executable||'instancia anterior'}). Reinicie ABRIR_PROGRAMA.bat; debe usar .venv-web.`:'BO (GP + EI), TPE y búsqueda aleatoria disponibles en el servidor.';
      if(lastData)render(lastData);
      else show(info.enabled?'Disponible para la conversión actual.':info.disabled_reason==='PUBLIC_ACCESS'?'La instancia pública tiene la optimización desactivada. Utilice la aplicación local.':'Abra ABRIR_PROGRAMA.bat y cierre cualquier servidor local anterior para habilitar la optimización.');readiness();
    }catch(error){info=null;controls();show(error.message);}
  }
  $('optimization-refresh').onclick=refresh;
  document.addEventListener('xyzc:conversion-changed',readiness);refresh();
}
