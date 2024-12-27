import React from "react";
import Image from 'next/image';
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Badge } from "@/components/ui/badge";
import { Card } from "@/components/ui/card";

const Page = () => {
  const pedidos = [
    { id: "123456", status: "Pendente", data: "02/12/2024", color: "yellow" },
    { id: "379372", status: "Entregue", data: "01/12/2024", color: "green" },
    { id: "959849", status: "Entregue", data: "01/12/2024", color: "green" },
    { id: "123456", status: "Pendente", data: "01/12/2024", color: "yellow" },
    { id: "123456", status: "Cancelado", data: "30/11/2024", color: "red" },
    { id: "959849", status: "Cancelado", data: "30/11/2024", color: "red" },
    { id: "123456", status: "Pendente", data: "30/11/2024", color: "yellow" },
    { id: "123456", status: "Entregue", data: "30/11/2024", color: "green" },
    { id: "959849", status: "Entregue", data: "29/11/2024", color: "green" },
    { id: "959849", status: "Entregue", data: "29/11/2024", color: "green" },
  ];

  const badgeColors = {
    yellow: "bg-yellow-100 text-yellow-800",
    green: "bg-green-100 text-green-800",
    red: "bg-red-100 text-red-800",
  };

  return (
    <div className="p-6 bg-gray-100 min-h-screen">
      {/* Header */}
      <header className="flex items-center justify-between mb-8">
        <div className='justify-start'>
          <Image width={100} height={100} src="/logo_header.png" alt="logo" />
        </div>
        <nav className="flex space-x-10 px-20">
          <a href="#" className="text-sm text-gray-700 hover:underline">
            Realizar Pedido
          </a>
          <a href="#" className="text-sm text-gray-700 hover:underline">
            Visualizar dashboards
          </a>
          <a href="#" className="text-sm text-gray-700 hover:underline">
            Pontos de Entrega
          </a>
        </nav>
      </header>

      {/* Card de Pedidos */}
      <Card className="p-6 shadow-lg rounded-lg">
        <div className="flex justify-between mb-4">
          <h2 className="text-xl font-semibold">Fila de Pedidos</h2>
          <Input placeholder="Pesquisar..." className="w-1/3" />
        </div>

        {/* Cabeçalho da Tabela */}
        <div className="grid grid-cols-4 gap-4 items-center text-center font-medium">
          <span>Número do Pedido</span>
          <span>Status</span>
          <span>Data</span>
          <span>Filtrar por:</span>
        </div>

        {/* Lista de Pedidos */}
        <div className="divide-y divide-gray-200 mt-4">
          {pedidos.map((pedido, index) => (
            <div
              key={index}
              className="grid grid-cols-4 gap-4 items-center text-center py-2"
            >
              <span className="text-blue-600 font-semibold">
                {`Pedido ${pedido.id}`}
              </span>
              <Badge className={badgeColors[pedido.color]}>{pedido.status}</Badge>
              <span className="text-purple-600">{pedido.data}</span>
              <span></span>
            </div>
          ))}
        </div>
      </Card>

      {/* Botão Adicionar Novo Pedido */}
      <Button className="mt-6 bg-sky-500" variant="default">
        Fazer Novo Pedido
      </Button>
    </div>
  );
};

export default Page;
