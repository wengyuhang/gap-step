def norm:map(.*.)|add|sqrt;
def sub($b):[range(0;3) as $k|.[$k]-$b[$k]];
def dot($b):[range(0;3) as $k|.[$k]*$b[$k]]|add;
. as $scene | [.windows[]|. as $w| {name,center,sign,amplitude,frequency,swing,frame_radius:([.mesh[][]|norm]|max),motion_bound:(if .swing then 2*.length*(.amplitude/.length|asin|./2|sin) else .amplitude end),normal:[range(0;3) as $k|(-$w.sign)*$w.rotation[$k][2]]}] as $w |
{sampled:false, scope:"all-time triangle-inequality bound on distinct gate bodies; not drone trajectory safety",start:.start,goal:.goal,not_return_to_start:((.start|sub($scene.goal)|norm)>1e-8),
pairs:[range(0;($w|length)) as $i|range($i+1;($w|length)) as $j|{i:($i+1),j:($j+1),lower_separation_m:(($w[$i].center|sub($w[$j].center)|norm)-$w[$i].motion_bound-$w[$j].motion_bound-$w[$i].frame_radius-$w[$j].frame_radius)}],
chord_normals:[range(0;($w|length)) as $i|{gate:($i+1),incoming_projection:($w[$i].center|sub(if $i==0 then $scene.start else $w[$i-1].center end)|dot($w[$i].normal)),outgoing_projection:((if $i+1==($w|length) then $scene.goal else $w[$i+1].center end)|sub($w[$i].center)|dot($w[$i].normal))}]}
