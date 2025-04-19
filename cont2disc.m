%converting continuous value to discrete one to match the grid size

function DiscValue = cont2disc(ContValue)
cellSize=0.1;
DiscValue=round(ContValue/cellSize);
end