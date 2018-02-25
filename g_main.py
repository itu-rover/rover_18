from grapher import Grapher

lines = [ [[0.,0.,0.],[1.,1.,1.]] ]

G = Grapher(lines)
G.redraw(lines)

def update_lines():
    lines[0][0][0] += 0.01
    G.redraw(lines)

G.show(update_lines)
