"""Count x500 contacts only during the scored task, excluding launch/cleanup."""
import re


def parse_contacts(path, end_sim_s=70.):
    text = path.read_text(errors='replace') if path.exists() else ''
    messages = re.split(r'(?m)(?=^header \{)', text)
    pairs, stamps = [], []
    for message in messages:
        stamp = re.search(r'stamp\s*\{\s*sec:\s*(\d+)\s*nsec:\s*(\d+)', message)
        if stamp is None:
            continue
        at = int(stamp[1])+int(stamp[2])*1e-9
        if at < 30. or at > end_sim_s+1e-6:
            continue
        found = re.findall(r'collision1\s*\{.*?name:\s*"([^"]+)".*?\}\s*collision2\s*\{.*?name:\s*"([^"]+)"', message, re.S)
        found = [(a,b) for a,b in found if 'x500_0::' in a or 'x500_0::' in b]
        if found:
            stamps.append(at)
            pairs.extend(found)
    return dict(raw_log=path.name, contact_pair_count=len(pairs),
                first_contact_sim_time_s=min(stamps) if stamps else None,
                collision_pairs=sorted({f'{a} <-> {b}' for a,b in pairs}),
                interval_sim_s=[30.,end_sim_s])
