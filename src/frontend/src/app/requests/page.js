"use client";

import React, { useState } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Checkbox } from "@/components/ui/checkbox";

import Image from 'next/image';

const PedidoPage = () => {
  const [newOption, setNewOption] = useState("");
  const [selectedItems, setSelectedItems] = useState({
    Remédio: false,
    Seringa: true,
    Vacinas: false,
    Toalhas: false,
  });
  const [selectedLocations, setSelectedLocations] = useState({
    Enfermaria: false,
    "Quarto 1": true,
    "Quarto 2": false,
    Sala: false,
  });

  const handleCheckboxChange = (type, key) => {
    if (type === "items") {
      setSelectedItems({ ...selectedItems, [key]: !selectedItems[key] });
    } else if (type === "locations") {
      setSelectedLocations({
        ...selectedLocations,
        [key]: !selectedLocations[key],
      });
    }
  };

  const handleAddOption = () => {
    if (newOption.trim()) {
      setSelectedItems({ ...selectedItems, [newOption]: false });
      setNewOption("");
    }
  };

  return (
    <div className="min-h-screen bg-gray-50 p-3">
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

      <div className="grid grid-cols-1 md:grid-cols-2 px-28 ">
        {/* Lado Esquerdo */}
        <div>
          <h2 className="text-lg mb-4">Selecione os pedidos a serem feitos:</h2>
          <div className="space-y-2">
            {Object.keys(selectedItems).map((item) => (
              <label key={item} className="flex items-center space-x-2">
                <Checkbox 
                  checked={selectedItems[item]}
                  onCheckedChange={() => handleCheckboxChange("items", item)}
                  className="data-[state=checked]:bg-[#1282A2] rounded-md border-zinc-400 focus:border-[#1282A2]"
                />
                <span className="text-sm">{item}</span>
              </label>
            ))}
          </div>

          <h3 className="text-sm mt-4">Adicionar nova opção:</h3>
          <div className="flex items-center space-x-2 mt-2">
            <Input className='h-16 w-96  focus:border-0'
              placeholder="Digite uma opção"
              value={newOption}
              onChange={(e) => setNewOption(e.target.value)}
            />
          </div>
          <div>
            <Button onClick={handleAddOption} className="bg-[#1282A2] text-white mt-4 rounded-full">
              Adicionar
            </Button>
          </div>

          <h2 className="text-lg mt-8 mb-4">Selecione o local a ser percorrido:</h2>
          <div className="space-y-2">
            {Object.keys(selectedLocations).map((location) => (
              <label key={location} className="flex items-center space-x-2">
                <Checkbox
                  checked={selectedLocations[location]}
                  onCheckedChange={() => handleCheckboxChange("locations", location)}
                  className="data-[state=checked]:bg-[#1282A2] rounded-md border-zinc-400 focus:border-[#1282A2]"
                />
                <span className="text-sm">{location}</span>
              </label>
            ))}
          </div>
          <div className="px-28">
          <Button className="mt-6 text-white rounded-full py-6 w-52 h-7 flex justify-center items-center text-sm bg-[#1282A2]">
            Fazer Novo Pedido
          </Button>
          </div>
        </div>

        {/* Lado Direito */}
        <div className="w-auto">
          <h2 className="text-lg mb-4">Fila de Pedidos</h2>
          <div className="border-zinc-950 rounded-lg shadow p-3">
            {[
              { id: "123456", status: "Pendente" },
              { id: "379372", status: "Entregue" },
              { id: "959849", status: "Entregue" },
              { id: "123456", status: "Cancelado" },
              { id: "123456", status: "Pendente" },
              { id: "959849", status: "Cancelado" },
            ].map((pedido, index) => (
              <div
                key={index}
                className="flex justify-between items-center p-2 rounded-lg mb-2"
                style={{
                  backgroundColor:
                    pedido.status === "Pendente"
                      ? "#fffae6"
                      : pedido.status === "Entregue"
                      ? "#e6ffed"
                      : "#ffe6e6",
                }}
              >
                <span className="text-sm text-gray-800">Pedido {pedido.id}</span>
                <span
                  className={`text-xs font-bold ${
                    pedido.status === "Pendente"
                      ? "text-yellow-600"
                      : pedido.status === "Entregue"
                      ? "text-green-600"
                      : "text-red-600"
                  }`}
                >
                  {pedido.status}
                </span>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
};

export default PedidoPage;
