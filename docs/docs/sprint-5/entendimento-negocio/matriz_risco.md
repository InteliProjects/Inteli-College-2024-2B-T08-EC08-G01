---
custom_edit_url: null
---

# Matriz de risco

## Introdução

A matriz de risco consiste em uma ferramenta utilizada em análise de negócios para estudar a probabilidade de ocorrência de um evento (risco) e qual o grau de impacto associado a ele na solução, sendo uma pesquisa importante para identificar possíveis pontos de falha antes do desenvolvimento e assegurar as oportunidades previstas.

## Matriz de risco

![Matriz](/img/matriz_risco.png)

A tabela acima apresenta os riscos identificados no projeto sendo eles:

### Ameaças:

- *Incapacidade de reconhecer um comando (baixo, 30%):* como utilizaremos um modelo LLM (Large Language Mode) para transformar os comandos do usuário para as ações reconhecidas pelo sistema, há a possibilidade do comando não ser convertido para o padrão esperado ou não ser reconhecido, porém, a probabilidade de isto acontecer com um modelo comercial é pequena e, neste caso, basta o usuário enviar o comando novamente, ou seja, o impacto na operação é baixo.

- *Colisão com as pessoas do ambiente (baixo, 70%):* o robô utilizado na solução é pequeno e existe uma latência mínima da comunicação entre o LIDAR (Light Detection and Ranging), o processamento da informação e a instrução executada, contudo, ainda que ocorra, não acarrtará em grandes atrasos na operação nem machucados sérios. Assim, em um ambiente aberto com diversas pessoas andando, há uma chance elevada de uma colisão acontecer, mas seu impacto é baixo.
    
- *Prender-se no trajeto (médio, 10%):* por introduzirmos uma cesta no robô para fazer o transporte dos itens, há a chance do cálculo da trajetória causar um impedimento no caminho, prendendo o robô em algum local. Este risco afeta a operação pois depende da identificação do problema por um paciente ou colaborador para ser resolvido, toda via, como podemos alterar no código a distância mínima (clearance), a probabilidade deste risco ocorrer é muito baixa.

- *Perda / duplicação de comandos (médio, 50%):* a solução possuirá um sistema de fila para poder receber mais de um comando por vez, mas exite a possibilidade de um comando enviado para a fila ser perdido ou duplicado por problemas na conexão do usuáro, receber mais de um pacote simultaneamente, entre outros possíveis problemas, possuindo uma chance média de ocorrer e além de um efeito médio na operação do hospital.

- *Dessincronização com a simulação de mapeamento (muito alto, 10%):* por utilizar uma simulação do ambiente para criar o trajeto, há a possibilidade do robô dessincronizar com o seu avatar na simulação por problemas de arredondamento a longo prazo, gerando conflito entre o caminho esperado e o caminho utilizado, possuindo um grande impacto na operação ao inutilizar o robô, porém, possuindo uma probablilidade muito baixa inicialmente (a probabilidade aumenta com o tempo).

- *Perda de conexão com o servidor (muito alto, 30%):* ao colocarmos o processamento dos pedidos no servidor antes de enviá-los para o robô, caso o robô perca a conexão com uma rede, ele não receberá os comandos e ficará inutilizado, afetando drasticamente a operação do hospital, contudo, possuindo uma probabilidade baixa de ocorrer em um ambiente controlado.

### Oportunidades:

- *Aceleração do trânsito de itens (médio, 50%):* como a solução não depende de colaboradores disponíveis fazerem o transporte dos itens, a velocidade de transporte está relacionada ao número de robôs utilizados, desta maneira, há uma chance média de uma aceleração do transporte dentro do hospital.

- *Aumento na acessibilidade de fazer pedidos (médio, 90%):* por utilizarmos um STT (Speech To Text) para transformar comandos de áudio em texto e um LLM para transformar o texto recebido em um comando reconhecido pelo sistema, prevemos um aumento na acessibilidade do sistema de pedidos do hospital.

- *Agilidade do controle de entregas (alto, 30%):* o sistema registra em seu banco de dados informações sobre quem criou o chamado, local de origem, local de destino e os horários de criação do chamado e de conclusão, assim, agilizando o controle da movimentação de itens no hospital e tornando-se mais uma ferramenta de consulta em caso de falha na operação.

- *Otimização da mão de obra (muito alto, 70%):* ao fazer o transporte de materiais dentro do hospital, o robô assume esta função dos colaboradores e permite que esta a mão de obra seja alocada em outros processos mais complexos, valorizando o tempo dos colaboradores. Dessa maneira, existe uma probabilidade alta da implementação do robô afetar significativamente a operação ao otimizar o uso dos colaboradores.